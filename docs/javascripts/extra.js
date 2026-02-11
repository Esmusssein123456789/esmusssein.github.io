/**
 * 一念 · 首页打字机效果
 * 循环展示多条标语，逐字打出再逐字删除
 */
document.addEventListener("DOMContentLoaded", function () {
  const el = document.getElementById("typewriter-text");
  if (!el) return;

  const texts = [
    "欢迎来到一念 ✨",
    "记录 · 思考 · 成长",
    "Hello World 🌍",
    "Keep Learning, Keep Building",
  ];

  let textIdx = 0;
  let charIdx = 0;
  let isDeleting = false;

  function type() {
    const current = texts[textIdx];

    if (isDeleting) {
      charIdx--;
      el.textContent = current.substring(0, charIdx);
    } else {
      charIdx++;
      el.textContent = current.substring(0, charIdx);
    }

    // 打完 → 停留 2s → 开始删除
    if (!isDeleting && charIdx === current.length) {
      setTimeout(function () {
        isDeleting = true;
        type();
      }, 2000);
      return;
    }

    // 删完 → 切换下一条
    if (isDeleting && charIdx === 0) {
      isDeleting = false;
      textIdx = (textIdx + 1) % texts.length;
    }

    setTimeout(type, isDeleting ? 40 : 100);
  }

  type();
});
