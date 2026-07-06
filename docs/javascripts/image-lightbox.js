(function () {
  var overlay = null;

  function closeLightbox() {
    if (!overlay) return;
    overlay.classList.remove("is-open");
    document.body.classList.remove("img-lightbox-open");
    overlay.querySelector(".img-lightbox-image").removeAttribute("src");
  }

  function openLightbox(source) {
    if (!overlay) {
      overlay = document.createElement("div");
      overlay.className = "img-lightbox-overlay";
      overlay.innerHTML =
        '<button type="button" class="img-lightbox-close" aria-label="Close">×</button>' +
        '<img class="img-lightbox-image" alt="" />';

      document.body.appendChild(overlay);

      overlay.querySelector(".img-lightbox-close").addEventListener("click", closeLightbox);

      overlay.addEventListener("click", function (e) {
        if (e.target === overlay) closeLightbox();
      });

      /* Stop wheel from browser page zoom while lightbox is open (looks like image shrinks) */
      overlay.addEventListener(
        "wheel",
        function (e) {
          e.preventDefault();
        },
        { passive: false }
      );

      document.addEventListener("keydown", function (e) {
        if (e.key === "Escape") closeLightbox();
      });
    }

    var img = overlay.querySelector(".img-lightbox-image");
    img.src = source.currentSrc || source.src;
    img.alt = source.alt || "";

    overlay.classList.add("is-open");
    document.body.classList.add("img-lightbox-open");
  }

  document.addEventListener("DOMContentLoaded", function () {
    document.querySelectorAll("img.img-zoom").forEach(function (img) {
      img.setAttribute("title", "Click to view full screen");
      img.addEventListener("click", function () {
        openLightbox(img);
      });
    });
  });
})();
