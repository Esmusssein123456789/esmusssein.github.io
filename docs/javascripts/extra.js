/**
 * 一念 · 首页打字机效果
 * 逐行打印，全部打完后停止，不删除、不循环
 */
document.addEventListener("DOMContentLoaded", function () {
  var container = document.getElementById("typewriter-lines");
  if (!container) return;

  var lines = [
    "Keep Thinking, Keep Fighting",
  ];

  var lineIdx = 0;
  var charIdx = 0;
  var cursorSpan = null;

  var baseFontSize = 2; // rem
  var minFontSize = 1;  // rem

  function fitLine(lineEl) {
    var parentW = container.offsetWidth;
    if (parentW <= 0) return;
    // 先重置到基础字号测量
    lineEl.style.fontSize = baseFontSize + "rem";
    var textW = lineEl.scrollWidth;
    if (textW > parentW) {
      var newSize = Math.max(baseFontSize * (parentW / textW) * 0.98, minFontSize);
      lineEl.style.fontSize = newSize + "rem";
    }
  }

  function createLine() {
    var lineEl = document.createElement("div");
    lineEl.className = "typewriter-line";
    var textSpan = document.createElement("span");
    textSpan.className = "tw-text";
    cursorSpan = document.createElement("span");
    cursorSpan.className = "typewriter-cursor";
    cursorSpan.textContent = "|";
    lineEl.appendChild(textSpan);
    lineEl.appendChild(cursorSpan);
    container.appendChild(lineEl);
    return textSpan;
  }

  var currentTextEl = createLine();

  function type() {
    var currentLine = lines[lineIdx];

    if (charIdx <= currentLine.length) {
      currentTextEl.textContent = currentLine.substring(0, charIdx);
      fitLine(currentTextEl.parentElement);
      charIdx++;
      setTimeout(type, 100);
    } else {
      // 当前行打完，移除光标
      if (cursorSpan) cursorSpan.style.display = "none";

      lineIdx++;
      if (lineIdx < lines.length) {
        charIdx = 0;
        setTimeout(function () {
          currentTextEl = createLine();
          type();
        }, 600);
      }
      // 全部打完 → 停止
    }
  }

  type();

  // 窗口缩放时重新适配所有已打出的行
  var resizeTimer;
  window.addEventListener("resize", function () {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(function () {
      var allLines = container.querySelectorAll(".typewriter-line");
      allLines.forEach(function (line) { fitLine(line); });
    }, 120);
  });
});

/**
 * 站点统计 — 点击切换显示
 */
document.addEventListener("DOMContentLoaded", function () {
  var toggleBtn = document.getElementById("toggle-stats");
  var statsPanel = document.getElementById("stats-panel");
  if (!toggleBtn || !statsPanel) return;

  var loaded = false;

  toggleBtn.addEventListener("click", function (e) {
    e.preventDefault();
    var isHidden = statsPanel.style.display === "none";
    statsPanel.style.display = isHidden ? "block" : "none";

    if (isHidden && !loaded) {
      loaded = true;
      loadStats();
    }
  });

  function loadStats() {
    var SITE_CREATED = new Date("2026-02-11");
    var statsText = document.getElementById("stats-text");
    if (!statsText) return;

    // 页面总数
    var navLinks = document.querySelectorAll(".md-nav__link[href]");
    var uniquePages = new Set();
    navLinks.forEach(function (a) {
      var href = a.getAttribute("href");
      if (href && !href.startsWith("#") && !href.startsWith("http")) {
        uniquePages.add(href.replace(/\/$/, "").replace(/index\.html$/, ""));
      }
    });
    var pageCount = Math.max(uniquePages.size, 1);

    // 运行时间
    var now = new Date();
    var diffMs = now - SITE_CREATED;
    var days = Math.floor(diffMs / (1000 * 60 * 60 * 24));
    var hours = Math.floor((diffMs % (1000 * 60 * 60 * 24)) / (1000 * 60 * 60));
    var mins = Math.floor((diffMs % (1000 * 60 * 60)) / (1000 * 60));

    var uptimeStr;
    if (days > 365) {
      var years = Math.floor(days / 365);
      var remDays = days % 365;
      uptimeStr = years + "y " + remDays + "d " + hours + "h " + mins + "m";
    } else {
      uptimeStr = days + "d " + hours + "h " + mins + "m";
    }

    statsText.innerHTML =
      '<p class="stats-line">📄 页面总数: ' + pageCount + '</p>' +
      '<p class="stats-line">⏱ 运行时间: ' + uptimeStr + '</p>';

    // 字数统计（异步）
    fetch(window.location.origin + "/sitemap.xml")
      .then(function (r) { return r.text(); })
      .then(function (xml) {
        var parser = new DOMParser();
        var doc = parser.parseFromString(xml, "text/xml");
        var locs = doc.querySelectorAll("loc");
        var urls = [];
        locs.forEach(function (loc) { urls.push(loc.textContent); });

        return Promise.all(urls.map(function (url) {
          return fetch(url)
            .then(function (r) { return r.text(); })
            .then(function (html) {
              var tmp = document.createElement("div");
              tmp.innerHTML = html;
              var content = tmp.querySelector(".md-content");
              if (!content) return 0;
              var text = content.textContent || "";
              var cn = (text.match(/[\u4e00-\u9fff]/g) || []).length;
              var en = (text.match(/[a-zA-Z]+/g) || []).length;
              return cn + en;
            })
            .catch(function () { return 0; });
        }));
      })
      .then(function (counts) {
        var total = counts.reduce(function (a, b) { return a + b; }, 0);
        var wordStr = total > 10000
          ? (total / 10000).toFixed(1) + " 万"
          : total.toLocaleString();
        statsText.innerHTML =
          '<p class="stats-line">📄 页面总数: ' + pageCount + '</p>' +
          '<p class="stats-line">✏️ 总字数: ' + wordStr + '</p>' +
          '<p class="stats-line">⏱ 运行时间: ' + uptimeStr + '</p>';
      })
      .catch(function () { /* keep current text */ });
  }
});

/**
 * 页面统计栏 — 自动注入到每页 h1 标题下方
 * 统计：字数、代码块、图片、预计阅读时间、阅读量（不蒜子）
 */
document.addEventListener("DOMContentLoaded", function () {
  // 跳过首页（有 hero 打字机的页面）
  if (document.getElementById("typewriter-lines")) return;

  var article = document.querySelector(".md-content__inner");
  if (!article) return;

  var h1 = article.querySelector("h1");
  if (!h1) return;

  // 统计字数（中文字符 + 英文单词）
  // 克隆内容，移除代码块后统计纯正文
  var clone = article.cloneNode(true);
  var codeTags = clone.querySelectorAll("pre, code, .highlight");
  codeTags.forEach(function (el) { el.remove(); });
  var text = clone.textContent || "";
  var cnCount = (text.match(/[\u4e00-\u9fff]/g) || []).length;
  var enCount = (text.match(/[a-zA-Z]+/g) || []).length;
  var wordCount = cnCount + enCount;

  // 图片数量
  var imgCount = article.querySelectorAll("img").length;

  // 代码块数量
  var codeCount = article.querySelectorAll("pre > code").length;

  // 阅读时间（中文 300 字/分钟 + 英文 200 词/分钟）
  var readMinutes = Math.ceil(cnCount / 300 + enCount / 200);
  if (readMinutes < 1) readMinutes = 1;

  // 构建统计栏
  var bar = document.createElement("div");
  bar.className = "page-stats-bar";

  var items = [];
  items.push('<span class="ps-item"><i class="ps-icon">✏</i> 约 ' + wordCount + ' 个字</span>');

  if (codeCount > 0) {
    items.push('<span class="ps-item"><i class="ps-icon">💻</i> ' + codeCount + ' 个代码块</span>');
  }
  if (imgCount > 0) {
    items.push('<span class="ps-item"><i class="ps-icon">🖼</i> ' + imgCount + ' 张图片</span>');
  }

  items.push('<span class="ps-item"><i class="ps-icon">⏱</i> 预计阅读时间 ' + readMinutes + ' 分钟</span>');

  // 不蒜子阅读量占位
  items.push('<span class="ps-item"><i class="ps-icon">👁</i> 总阅读量 <span id="busuanzi_value_page_pv">-</span> 次</span>');

  bar.innerHTML = items.join("");

  // 插入到 h1 之后
  h1.parentNode.insertBefore(bar, h1.nextSibling);
});
