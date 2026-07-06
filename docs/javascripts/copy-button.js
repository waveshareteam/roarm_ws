document.addEventListener("DOMContentLoaded", function () {
  document.querySelectorAll("div.highlight").forEach(function (block) {
    var pre = block.querySelector("pre");
    if (!pre) return;

    var btn = document.createElement("button");
    btn.type = "button";
    btn.className = "copy-code-button";
    btn.setAttribute("aria-label", "Copy code");
    btn.title = "Copy to clipboard";
    btn.innerHTML =
      '<svg xmlns="http://www.w3.org/2000/svg" width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="9" y="9" width="13" height="13" rx="2"/><path d="M5 15H4a2 2 0 0 1-2-2V4a2 2 0 0 1 2-2h9a2 2 0 0 1 2 2v1"/></svg>';

    btn.addEventListener("click", function () {
      var text = pre.innerText.replace(/\n$/, "");
      function onSuccess() {
        btn.classList.add("copied");
        btn.title = "Copied!";
        setTimeout(function () {
          btn.classList.remove("copied");
          btn.title = "Copy to clipboard";
        }, 1500);
      }
      if (navigator.clipboard && navigator.clipboard.writeText) {
        navigator.clipboard.writeText(text).then(onSuccess).catch(fallback);
      } else {
        fallback();
      }
      function fallback() {
        var ta = document.createElement("textarea");
        ta.value = text;
        ta.style.position = "fixed";
        ta.style.left = "-9999px";
        document.body.appendChild(ta);
        ta.select();
        try {
          document.execCommand("copy");
          onSuccess();
        } catch (e) {
          btn.title = "Copy failed";
        }
        document.body.removeChild(ta);
      }
    });

    block.appendChild(btn);
  });
});
