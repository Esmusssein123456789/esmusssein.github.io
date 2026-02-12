/**
 * 一念 · 首页打字机效果
 * 自动根据容器宽度缩放字号，确保文字永远不会溢出
 */
document.addEventListener("DOMContentLoaded", function () {
  var el = document.getElementById("typewriter-text");
  if (!el) return;

  var heading = el.closest("h1");
  if (!heading) return;

  var texts = [
    "一念起，万水千山",
    "Keep Thinking, Keep Fighting",
    "Stay Foolish, Stay Hungry",
  ];

  var baseFontSize = 2.6; // rem
  var minFontSize = 1.2;  // rem

  // 动态调整字号，保证文字不溢出
  function fitText() {
    if (!heading) return;
    var container = heading.parentElement;
    if (!container) return;

    // 先设为基础字号来测量
    heading.style.fontSize = baseFontSize + "rem";
    var containerW = container.offsetWidth - 20; // 留一点余量
    var textW = heading.scrollWidth;

    if (textW > containerW && containerW > 0) {
      var ratio = containerW / textW;
      var newSize = Math.max(baseFontSize * ratio, minFontSize);
      heading.style.fontSize = newSize + "rem";
    }
  }

  var textIdx = 0;
  var charIdx = 0;
  var isDeleting = false;

  function type() {
    var current = texts[textIdx];

    if (isDeleting) {
      charIdx--;
      el.textContent = current.substring(0, charIdx);
    } else {
      charIdx++;
      el.textContent = current.substring(0, charIdx);
    }

    fitText();

    if (!isDeleting && charIdx === current.length) {
      setTimeout(function () {
        isDeleting = true;
        type();
      }, 2000);
      return;
    }

    if (isDeleting && charIdx === 0) {
      isDeleting = false;
      textIdx = (textIdx + 1) % texts.length;
      // 切换时重置字号
      heading.style.fontSize = baseFontSize + "rem";
    }

    setTimeout(type, isDeleting ? 40 : 100);
  }

  type();

  // 窗口缩放时也重新适配
  var resizeTimer;
  window.addEventListener("resize", function () {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(fitText, 100);
  });
});

/**
 * 站点统计 — 点击展开一行文字
 */
document.addEventListener("DOMContentLoaded", function () {
  var SITE_CREATED = new Date("2026-02-11");
  var statsText = document.getElementById("stats-text");
  if (!statsText) return;

  // 笔记总数
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
    uptimeStr = years + " 年 " + remDays + " 天 " + hours + " 小时 " + mins + " 分钟";
  } else {
    uptimeStr = days + " 天 " + hours + " 小时 " + mins + " 分钟";
  }

  statsText.textContent =
    "页面总数: " + pageCount +
    " / 网站运行时间: " + uptimeStr;

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
      statsText.textContent =
        "页面总数: " + pageCount +
        " / 总字数: " + wordStr +
        " / 网站运行时间: " + uptimeStr;
    })
    .catch(function () { /* keep current text */ });
});
