#include <memory>
#include <math.h>
#include <array>
#include <cmath>

namespace roarm_m2 {

#define ARM_L1_LENGTH_MM    126.06
#define ARM_L2_LENGTH_MM_A  236.82
#define ARM_L2_LENGTH_MM_B  30.00 
#define ARM_L3_LENGTH_MM_A_0  280.15 //214 
#define ARM_L3_LENGTH_MM_B_0  1.73
#define ARM_L4_LENGTH_MM_A  67.85
#define ARM_L4_LENGTH_MM_B  5.98

static double l1  = ARM_L1_LENGTH_MM;
static double l2A = ARM_L2_LENGTH_MM_A;
static double l2B = ARM_L2_LENGTH_MM_B;
static double l2  = sqrt(l2A * l2A + l2B * l2B);
static double t2rad = atan2(l2B, l2A);
static double l3A = ARM_L3_LENGTH_MM_A_0;
static double l3B = ARM_L3_LENGTH_MM_B_0;
static double l3  = sqrt(l3A * l3A + l3B * l3B);
static double t3rad = atan2(l3B, l3A);

static double BASE_point_RAD = 0;
static double SHOULDER_point_RAD = 0;
static double ELBOW_point_RAD = M_PI / 2;
static double EOAT_point_RAD = M_PI;
static double EOAT_point_RAD_BUFFER;

static double BASE_point_ANG  = 0;
static double SHOULDER_point_ANG = 0;
static double ELBOW_point_ANG = 90.0;
static double EOAT_point_ANG  = 180.0;

static bool nanIK;
static bool nanFK;
static double base_r;
static bool EEMode = 0;

static double EoAT_A = 0;
static double EoAT_B = 0;
static double l4A = ARM_L4_LENGTH_MM_A;
static double l4B = ARM_L4_LENGTH_MM_B;
static double lEA = EoAT_A + ARM_L4_LENGTH_MM_A;
static double lEB = EoAT_B + ARM_L4_LENGTH_MM_B;
static double lE  = sqrt(lEA * lEA + lEB * lEB);
static double tErad = atan2(lEB, lEA);

static double initX = l3A+l2B; //
static double initY = 0;
static double initZ = l2A-l3B;
static double initT = M_PI;

static double goalX = initX;
static double goalY = initY;
static double goalZ = initZ;
static double goalT = initT;

static double lastX = goalX;
static double lastY = goalY;
static double lastZ = goalZ;
static double lastT = goalT;

std::array<double, 2> simpleLinkageIkRad(double aIn, double bIn) {
    double psi, alpha, omega, beta, L2C, LC, lambda, delta, LA, LB;
    LA = l2;
    LB = l3;
    if (fabs(bIn) < 1e-6) {
        psi = acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn)) + t2rad;
        alpha = M_PI / 2.0 - psi;
        omega = acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB));
        beta = psi + omega - t3rad;
    } else {
        L2C = aIn * aIn + bIn * bIn;
        LC = sqrt(L2C);
        lambda = atan2(bIn, aIn);
        psi = acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) + t2rad;
        alpha = M_PI / 2.0 - lambda - psi;
        omega = acos((LB * LB + L2C - LA * LA) / (2 * LC * LB));
        beta = psi + omega - t3rad;
    }

    delta = M_PI / 2.0 - alpha - beta;

    SHOULDER_point_RAD = alpha;
    ELBOW_point_RAD    = beta;
    EOAT_point_RAD_BUFFER  = delta;

    nanIK = isnan(alpha) || isnan(beta) || isnan(delta);
    std::array<double, 2> result = {SHOULDER_point_RAD, ELBOW_point_RAD};

    return result;
}

std::array<double, 2> cartesian_to_polar(double x, double y) {
    double r = sqrt(x * x + y * y);
    double theta = atan2(y, x);
    std::array<double, 2> result = {r, theta};
    return result;
}

void polarToCartesian(double r, double theta, double &x, double &y) {
  x = r * cos(theta);
  y = r * sin(theta);
}

std::array<double, 5>  computePosbyJointRad(double base_joint_rad, double shoulder_joint_rad, double elbow_joint_rad, double hand_joint_rad) {
  if (EEMode == 0) {
    // the end of the arm.
    double r_ee, x_ee, y_ee, z_ee;

    // compute the end position of the first linkage(the linkage between baseJoint and shoulderJoint).
    double aOut, bOut, cOut, dOut, eOut, fOut;

    polarToCartesian(l2, ((M_PI / 2) - (shoulder_joint_rad + t2rad)), aOut, bOut);
    polarToCartesian(l3, ((M_PI / 2) - (elbow_joint_rad + shoulder_joint_rad)), cOut, dOut);

    r_ee = aOut + cOut;
    z_ee = bOut + dOut;
    
    polarToCartesian(r_ee, base_joint_rad, eOut, fOut);
    x_ee = eOut;
    y_ee = fOut;

    lastX = x_ee;
    lastY = y_ee;
    lastZ = z_ee;
  }
  else if (EEMode == 1) {
    double aOut, bOut,   cOut, dOut,   eOut, fOut,   gOut, hOut;
    double r_ee, z_ee;

    polarToCartesian(l2, ((M_PI / 2) - (shoulder_joint_rad + t2rad)), aOut, bOut);
    polarToCartesian(l3, ((M_PI / 2) - (elbow_joint_rad + shoulder_joint_rad + t3rad)), cOut, dOut);
    polarToCartesian(lE, -((hand_joint_rad + tErad) - M_PI - (M_PI/2 - shoulder_joint_rad - elbow_joint_rad)), eOut, fOut);

    r_ee = aOut + cOut + eOut;
    z_ee = bOut + dOut + fOut;

    polarToCartesian(r_ee, base_joint_rad, gOut, hOut);

    lastX = gOut;
    lastY = hOut;
    lastZ = z_ee;
    lastT = hand_joint_rad - (M_PI - shoulder_joint_rad - elbow_joint_rad) + (M_PI / 2);
  }
  std::array<double, 5> result = {lastX,lastY,lastZ,EEMode,lastT};
  return result;
}

std::vector<double> computeJointRadbyPos(double x, double y, double z, double t ) {
    std::array<double, 2> bases = cartesian_to_polar(x,y);
    std::array<double, 2> radians = simpleLinkageIkRad(bases[0], z);
    
    std::vector<double> result = {bases[1], radians[0], radians[1]};
    return result;
}

} // namespace roarm_m2

namespace roarm_m3 {

#define ARM_L1_LENGTH_MM    126.06
#define ARM_L2_LENGTH_MM_A  236.82
#define ARM_L2_LENGTH_MM_B  30.00 
#define ARM_L3_LENGTH_MM_A_0	144.49
#define ARM_L3_LENGTH_MM_B_0	0
#define ARM_L3_LENGTH_MM_A_1	144.49
#define ARM_L3_LENGTH_MM_B_1	0
#define ARM_L4_LENGTH_MM_A 	171.67
#define ARM_L4_LENGTH_MM_B  13.69

static double l1  = ARM_L1_LENGTH_MM;
static double l2A = ARM_L2_LENGTH_MM_A;
static double l2B = ARM_L2_LENGTH_MM_B;
static double l2  = sqrt(l2A * l2A + l2B * l2B);
static double t2rad = atan2(l2B, l2A);
static double l3A = ARM_L3_LENGTH_MM_A_0;
static double l3B = ARM_L3_LENGTH_MM_B_0;
static double l3  = sqrt(l3A * l3A + l3B * l3B);
static double t3rad = atan2(l3B, l3A);
static double EoAT_A = 0;
static double EoAT_B = 0;
static double l4A = ARM_L4_LENGTH_MM_A;
static double l4B = ARM_L4_LENGTH_MM_B;
static double lEA = EoAT_A + ARM_L4_LENGTH_MM_A;
static double lEB = EoAT_B + ARM_L4_LENGTH_MM_B;
static double lE  = sqrt(lEA * lEA + lEB * lEB);
static double tErad = atan2(lEB, lEA);

static double BASE_JOINT_RAD = 0;
static double SHOULDER_JOINT_RAD = 0;
static double ELBOW_JOINT_RAD = M_PI/2;
static double WRIST_JOINT_RAD = 0;
static double ROLL_JOINT_RAD = 0;
static double EOAT_JOINT_RAD = M_PI;
static double EOAT_JOINT_RAD_BUFFER;

static double BASE_JOINT_ANG  = 0;
static double SHOULDER_JOINT_ANG = 0;
static double ELBOW_JOINT_ANG = 90.0;
static double WRIST_JOINT_ANG = 0;
static double ROLL_JOINT_ANG = 0;
static double EOAT_JOINT_ANG  = 180.0;

static bool nanIK;
static bool nanFK;
static double base_r;
static double delta_x;
static double delta_y;
static double beta_x;
static double beta_y;

static double initX = l2B + l3A + ARM_L4_LENGTH_MM_A; //
static double initY = 0;
static double initZ = l2A - ARM_L4_LENGTH_MM_B;
static double initT = 0;
static double initR = 0;
static double initG = 3.14;

static double goalX = initX;
static double goalY = initY;
static double goalZ = initZ;
static double goalT = initT;
static double goalR = initR;
static double goalG = initG;


static double lastX = goalX;
static double lastY = goalY;
static double lastZ = goalZ;
static double lastT = goalT;
static double lastR = goalR;
static double lastG = goalG;

std::array<double, 3> simpleLinkageIkRad(double aIn, double bIn) {
  double psi, alpha, omega, beta, L2C, LC, lambda, delta, LA, LB;
  LA = l2;
  LB = l3;
  if (fabs(bIn) < 1e-6) {
    psi = acos((LA * LA + aIn * aIn - LB * LB) / (2 * LA * aIn)) + t2rad;
    alpha = M_PI / 2.0 - psi;
    omega = acos((aIn * aIn + LB * LB - LA * LA) / (2 * aIn * LB));
    beta = psi + omega - t3rad;
  } else {
    L2C = aIn * aIn + bIn * bIn;
    LC = sqrt(L2C);
    lambda = atan2(bIn, aIn);
    psi = acos((LA * LA + L2C - LB * LB) / (2 * LA * LC)) + t2rad;
    alpha = M_PI / 2.0 - lambda - psi;
    omega = acos((LB * LB + L2C - LA * LA) / (2 * LC * LB));
    beta = psi + omega - t3rad;
  }

  delta = M_PI / 2.0 - alpha - beta;

  SHOULDER_JOINT_RAD = alpha;
  ELBOW_JOINT_RAD    = beta;
  EOAT_JOINT_RAD_BUFFER  = delta;

  nanIK = isnan(alpha) || isnan(beta) || isnan(delta);
  std::array<double, 3> result = {SHOULDER_JOINT_RAD, ELBOW_JOINT_RAD, EOAT_JOINT_RAD_BUFFER};
  return result;
}

std::array<double, 2> rotatePoint(double theta) {
  double alpha = tErad + theta;
  double xB, yB;
  xB = -lE * cos(alpha);
  yB = -lE * sin(alpha);
  std::array<double, 2> result = {xB, yB};
  return result;
}

std::array<double, 2> movePoint(double xA, double yA, double s) {
  double distance = sqrt(pow(xA, 2) + pow(yA, 2)); 
  double xB, yB;
  if(distance - s <= 1e-6) {
    xB = 0; 
    yB = 0;
  }
  else {
    double ratio = (distance - s) / distance;
    xB = xA * ratio;
    yB = yA * ratio;
  }
  std::array<double, 2> result = {xB, yB};
  return result;  
}

std::array<double, 2> cartesian_to_polar(double x, double y) {
    double r = sqrt(x * x + y * y);
    double theta = atan2(y, x);
    std::array<double, 2> result = {r, theta};
    return result;
}

void polarToCartesian(double r, double theta, double &x, double &y) {
  x = r * cos(theta);
  y = r * sin(theta);
}

std::array<double, 5> computePosbyJointRad(double base_joint_rad, double shoulder_joint_rad, double elbow_joint_rad, double wrist_joint_rad, double roll_joint_rad, double hand_joint_rad) {
  double aOut, bOut,   cOut, dOut,   eOut, fOut,   gOut, hOut;
  double r_ee, z_ee;

  polarToCartesian(l2, ((M_PI / 2) - (shoulder_joint_rad + t2rad)), aOut, bOut);
  polarToCartesian(l3, ((M_PI / 2) - (elbow_joint_rad + shoulder_joint_rad + t3rad)), cOut, dOut);
  polarToCartesian(lE, ((M_PI / 2) - (elbow_joint_rad + shoulder_joint_rad + wrist_joint_rad + tErad)), eOut, fOut);

  r_ee = aOut + cOut + eOut;
  z_ee = bOut + dOut + fOut;

  polarToCartesian(r_ee, base_joint_rad, gOut, hOut);

  lastX = gOut;
  lastY = hOut;
  lastZ = z_ee;
  lastT = (elbow_joint_rad + shoulder_joint_rad + wrist_joint_rad) - M_PI/2;
  std::array<double, 5> result = {lastX,lastY,lastZ,roll_joint_rad,lastT};
  return result;
}

std::vector<double> computeJointRadbyPos(double x, double y, double z, double roll, double pitch ) {

    std::array<double, 2> delta = rotatePoint((pitch - 3.1416));
    std::array<double, 2> beta = movePoint(x, y, delta[0]);
    std::array<double, 2> bases = cartesian_to_polar(beta[0], beta[1]);
    std::array<double, 3> radians = simpleLinkageIkRad(bases[0], z + delta[1]);
    double WRIST_JOINT_RAD = radians[2] + pitch;
    double ROLL_JOINT_RAD = roll;
    
    std::vector<double> result = {bases[1], radians[0], radians[1], WRIST_JOINT_RAD, ROLL_JOINT_RAD};
    return result;
}

} // namespace roarm_m3
