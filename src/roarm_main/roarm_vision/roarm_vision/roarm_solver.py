import math

class RoArmM2:
    ARM_L1_LENGTH_MM = 126.06
    ARM_L2_LENGTH_MM_A = 236.82
    ARM_L2_LENGTH_MM_B = 30.00
    ARM_L3_LENGTH_MM_A_0 = 280.15
    ARM_L3_LENGTH_MM_B_0 = 1.73
    ARM_L4_LENGTH_MM_A = 67.85
    ARM_L4_LENGTH_MM_B = 5.98

    def __init__(self):
        self.l1 = self.ARM_L1_LENGTH_MM
        self.l2A = self.ARM_L2_LENGTH_MM_A
        self.l2B = self.ARM_L2_LENGTH_MM_B
        self.l2 = math.sqrt(self.l2A ** 2 + self.l2B ** 2)
        self.t2rad = math.atan2(self.l2B, self.l2A)

        self.l3A = self.ARM_L3_LENGTH_MM_A_0
        self.l3B = self.ARM_L3_LENGTH_MM_B_0
        self.l3 = math.sqrt(self.l3A ** 2 + self.l3B ** 2)
        self.t3rad = math.atan2(self.l3B, self.l3A)

        self.l4A = self.ARM_L4_LENGTH_MM_A
        self.l4B = self.ARM_L4_LENGTH_MM_B

        self.EOAT_point_RAD_BUFFER = 0
        self.nanIK = False
        self.EEMode = 0

        self.EoAT_A = 0
        self.EoAT_B = 0
        self.lEA = self.EoAT_A + self.l4A
        self.lEB = self.EoAT_B + self.l4B
        self.lE = math.sqrt(self.lEA ** 2 + self.lEB ** 2)
        self.tErad = math.atan2(self.lEB, self.lEA)

        self.lastX = self.l3A + self.l2B
        self.lastY = 0
        self.lastZ = self.l2A - self.l3B
        self.lastT = math.pi

    def polar_to_cartesian(self, r, theta):
        return r * math.cos(theta), r * math.sin(theta)

    def cartesian_to_polar(self, x, y):
        return math.sqrt(x ** 2 + y ** 2), math.atan2(y, x)

    def simple_linkage_ik_rad(self, aIn, bIn):
        def safe_acos(v):
            return math.acos(max(-1.0, min(1.0, v)))

        LA = self.l2
        LB = self.l3

        if abs(bIn) < 1e-6:
            psi = safe_acos((LA ** 2 + aIn ** 2 - LB ** 2) / (2 * LA * aIn)) + self.t2rad
            alpha = math.pi / 2 - psi
            omega = safe_acos((aIn ** 2 + LB ** 2 - LA ** 2) / (2 * aIn * LB))
            beta = psi + omega - self.t3rad
        else:
            L2C = aIn ** 2 + bIn ** 2
            LC = math.sqrt(L2C)
            lambd = math.atan2(bIn, aIn)
            psi = safe_acos((LA ** 2 + L2C - LB ** 2) / (2 * LA * LC)) + self.t2rad
            alpha = math.pi / 2 - lambd - psi
            omega = safe_acos((LB ** 2 + L2C - LA ** 2) / (2 * LC * LB))
            beta = psi + omega - self.t3rad

        delta = math.pi / 2 - alpha - beta
        self.EOAT_point_RAD_BUFFER = delta
        self.nanIK = math.isnan(alpha) or math.isnan(beta) or math.isnan(delta)
        return alpha, beta

    def compute_pos_by_joint_rad(self, baseRad, shoulderRad, elbowRad, handRad):
        if self.EEMode == 0:
            aOut, bOut = self.polar_to_cartesian(self.l2, math.pi / 2 - (shoulderRad + self.t2rad))
            cOut, dOut = self.polar_to_cartesian(self.l3, math.pi / 2 - (elbowRad + shoulderRad + self.t3rad))
            r_ee = aOut + cOut
            z_ee = bOut + dOut
            x_ee, y_ee = self.polar_to_cartesian(r_ee, baseRad)
            self.lastX, self.lastY, self.lastZ = x_ee, y_ee, z_ee
        elif self.EEMode == 1:
            aOut, bOut = self.polar_to_cartesian(self.l2, math.pi / 2 - (shoulderRad + self.t2rad))
            cOut, dOut = self.polar_to_cartesian(self.l3, math.pi / 2 - (elbowRad + shoulderRad + self.t3rad))
            eOut, fOut = self.polar_to_cartesian(self.lE, -((handRad + self.tErad) - math.pi - (math.pi / 2 - shoulderRad - elbowRad)))
            r_ee = aOut + cOut + eOut
            z_ee = bOut + dOut + fOut
            x_ee, y_ee = self.polar_to_cartesian(r_ee, baseRad)
            self.lastX, self.lastY, self.lastZ = x_ee, y_ee, z_ee
            self.lastT = handRad - (math.pi - shoulderRad - elbowRad) + math.pi / 2

        return self.lastX, self.lastY, self.lastZ, self.EEMode, self.lastT

    def compute_joint_rad_by_pos(self, x, y, z, t):
        r, theta = self.cartesian_to_polar(x, y)
        shoulder, elbow = self.simple_linkage_ik_rad(r, z)
        return theta, shoulder, elbow

    def set_EEMode(self, mode):
        self.EEMode = mode

    def get_nanIK(self):
        return self.nanIK



class RoArmM3:
    ARM_L1_LENGTH_MM = 126.06
    ARM_L2_LENGTH_MM_A = 236.82
    ARM_L2_LENGTH_MM_B = 30.0
    ARM_L3_LENGTH_MM_A_0 = 144.49
    ARM_L3_LENGTH_MM_B_0 = 0
    ARM_L3_LENGTH_MM_A_1 = 144.49
    ARM_L3_LENGTH_MM_B_1 = 0
    ARM_L4_LENGTH_MM_A = 171.67
    ARM_L4_LENGTH_MM_B = 13.69

    def __init__(self):
        self.l1 = self.ARM_L1_LENGTH_MM
        self.l2A = self.ARM_L2_LENGTH_MM_A
        self.l2B = self.ARM_L2_LENGTH_MM_B
        self.l2 = math.sqrt(self.l2A ** 2 + self.l2B ** 2)
        self.t2rad = math.atan2(self.l2B, self.l2A)
        self.l3A = self.ARM_L3_LENGTH_MM_A_0
        self.l3B = self.ARM_L3_LENGTH_MM_B_0
        self.l3 = math.sqrt(self.l3A ** 2 + self.l3B ** 2)
        self.t3rad = math.atan2(self.l3B, self.l3A)

        self.EoAT_A = 0
        self.EoAT_B = 0
        self.l4A = self.ARM_L4_LENGTH_MM_A
        self.l4B = self.ARM_L4_LENGTH_MM_B
        self.lEA = self.EoAT_A + self.l4A
        self.lEB = self.EoAT_B + self.l4B
        self.lE = math.sqrt(self.lEA ** 2 + self.lEB ** 2)
        self.tErad = math.atan2(self.lEB, self.lEA)

        self.SHOULDER_JOINT_RAD = 0
        self.ELBOW_JOINT_RAD = math.pi / 2
        self.EOAT_JOINT_RAD_BUFFER = None

        self.nanIK = False
        self.nanFK = False

        self.lastX = self.l2B + self.l3A + self.l4A
        self.lastY = 0
        self.lastZ = self.l2A - self.l4B
        self.lastT = 0
        self.lastR = 0
        self.lastG = 3.14

        self.lastValidResult = [0, 0, 0, 0, 0]

    def simple_linkage_ik_rad(self, aIn, bIn):
        def safe_acos(v):
            return math.acos(max(-1.0, min(1.0, v)))

        LA = self.l2
        LB = self.l3

        if abs(bIn) < 1e-6:
            psi = safe_acos((LA ** 2 + aIn ** 2 - LB ** 2) / (2 * LA * aIn)) + self.t2rad
            alpha = math.pi / 2 - psi
            omega = safe_acos((aIn ** 2 + LB ** 2 - LA ** 2) / (2 * aIn * LB))
            beta = psi + omega - self.t3rad
        else:
            L2C = aIn ** 2 + bIn ** 2
            LC = math.sqrt(L2C)
            lambd = math.atan2(bIn, aIn)
            psi = safe_acos((LA ** 2 + L2C - LB ** 2) / (2 * LA * LC)) + self.t2rad
            alpha = math.pi / 2 - lambd - psi
            omega = safe_acos((LB ** 2 + L2C - LA ** 2) / (2 * LC * LB))
            beta = psi + omega - self.t3rad

        delta = math.pi / 2 - alpha - beta

        # 更新关节状态
        self.SHOULDER_JOINT_RAD = alpha
        self.ELBOW_JOINT_RAD = beta
        self.EOAT_JOINT_RAD_BUFFER = delta

        # 检查是否有 NaN
        self.nanIK = math.isnan(alpha) or math.isnan(beta) or math.isnan(delta)

        return self.SHOULDER_JOINT_RAD, self.ELBOW_JOINT_RAD, self.EOAT_JOINT_RAD_BUFFER
        
    def rotate_point(self, theta):
        alpha = self.tErad + theta
        xB = -self.lE * math.cos(alpha)
        yB = -self.lE * math.sin(alpha)
        return xB, yB

    def move_point(self, xA, yA, s):
        distance = math.sqrt(xA ** 2 + yA ** 2)
        if distance - s <= 1e-6:
            return 0, 0
        else:
            ratio = (distance - s) / distance
            return xA * ratio, yA * ratio

    def cartesian_to_polar(self, x, y):
        r = math.sqrt(x ** 2 + y ** 2)
        theta = math.atan2(y, x)
        return r, theta

    def polar_to_cartesian(self, r, theta):
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        return x, y

    def compute_pos_by_joint_rad(self, base_joint_rad, shoulder_joint_rad, elbow_joint_rad, wrist_joint_rad, roll_joint_rad, hand_joint_rad):
        aOut, bOut = self.polar_to_cartesian(self.l2, math.pi / 2 - (shoulder_joint_rad + self.t2rad))
        cOut, dOut = self.polar_to_cartesian(self.l3, math.pi / 2 - (elbow_joint_rad + shoulder_joint_rad + self.t3rad))
        eOut, fOut = self.polar_to_cartesian(self.lE, math.pi / 2 - (elbow_joint_rad + shoulder_joint_rad + wrist_joint_rad + self.tErad))

        r_ee = aOut + cOut + eOut
        z_ee = bOut + dOut + fOut

        gOut, hOut = self.polar_to_cartesian(r_ee, base_joint_rad)

        self.lastX = gOut
        self.lastY = hOut
        self.lastZ = z_ee
        self.lastT = elbow_joint_rad + shoulder_joint_rad + wrist_joint_rad - math.pi / 2

        return self.lastX, self.lastY, self.lastZ, roll_joint_rad, self.lastT

    def compute_joint_rad_by_pos(self, x, y, z, roll, pitch, hand_joint_rad):
        delta = self.rotate_point(pitch - math.pi)
        beta = self.move_point(x, y, delta[0])
        bases = self.cartesian_to_polar(beta[0], beta[1])
        radians = self.simple_linkage_ik_rad(bases[0], z + delta[1])

        WRIST_JOINT_RAD = radians[2] + pitch
        ROLL_JOINT_RAD = roll

        result = [bases[1], radians[0], radians[1], WRIST_JOINT_RAD, ROLL_JOINT_RAD, hand_joint_rad]

        if all(not math.isnan(v) for v in result):
            self.lastValidResult = result
            return result
        else:
            print("Warning: Inverse kinematics returned NaN. Using last valid result.")
            return self.lastValidResult

    def get_nanIK(self):
        return self.nanIK
