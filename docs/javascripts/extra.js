/**
 * 一念 · 首页打字机效果
 * 循环展示多条标语，逐字打出再逐字删除
 */
document.addEventListener("DOMContentLoaded", function () {
  const el = document.getElementById("typewriter-text");
  if (!el) return;

  const texts = [
    "一念起 ✨",
    "Keep Thinking, Keep Fighting",
    "Stay Hungry, Stay Foolish",
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

/**
 * 站点统计
 * 遍历导航链接来计算页面数，计算运行天数等
 */
document.addEventListener("DOMContentLoaded", function () {
  // ── 站点创建日期（手动设定）──
  var SITE_CREATED = new Date("2026-02-11");

  // 笔记总数：统计导航中的所有页面链接
  var statPages = document.getElementById("stat-pages");
  if (statPages) {
    // 通过 sitemap 或导航链接计数
    var navLinks = document.querySelectorAll(".md-nav__link[href]");
    var uniquePages = new Set();
    navLinks.forEach(function (a) {
      var href = a.getAttribute("href");
      if (href && !href.startsWith("#") && !href.startsWith("http")) {
        // 标准化路径
        uniquePages.add(href.replace(/\/$/, "").replace(/index\.html$/, ""));
      }
    });
    // 至少显示已知的页面数
    var pageCount = Math.max(uniquePages.size, 1);
    statPages.textContent = pageCount;
  }

  // 总字数：通过 fetch 所有导航中的页面来统计
  var statWords = document.getElementById("stat-words");
  if (statWords) {
    statWords.textContent = "统计中...";
    // 获取 sitemap
    fetch(window.location.origin + "/sitemap.xml")
      .then(function (r) { return r.text(); })
      .then(function (xml) {
        var parser = new DOMParser();
        var doc = parser.parseFromString(xml, "text/xml");
        var locs = doc.querySelectorAll("loc");
        var urls = [];
        locs.forEach(function (loc) { urls.push(loc.textContent); });

        // 并行 fetch 所有页面，提取文本字数
        return Promise.all(urls.map(function (url) {
          return fetch(url)
            .then(function (r) { return r.text(); })
            .then(function (html) {
              var tmp = document.createElement("div");
              tmp.innerHTML = html;
              var content = tmp.querySelector(".md-content");
              if (!content) return 0;
              var text = content.textContent || "";
              // 中英混合字数统计
              var cn = (text.match(/[\u4e00-\u9fff]/g) || []).length;
              var en = (text.match(/[a-zA-Z]+/g) || []).length;
              return cn + en;
            })
            .catch(function () { return 0; });
        }));
      })
      .then(function (counts) {
        var total = counts.reduce(function (a, b) { return a + b; }, 0);
        if (total > 10000) {
          statWords.textContent = (total / 10000).toFixed(1) + " 万";
        } else if (total > 1000) {
          statWords.textContent = (total / 1000).toFixed(1) + "k";
        } else {
          statWords.textContent = total;
        }
      })
      .catch(function () {
        statWords.textContent = "—";
      });
  }

  // 创建时间
  var statCreated = document.getElementById("stat-created");
  if (statCreated) {
    statCreated.textContent = SITE_CREATED.toISOString().split("T")[0];
  }

  // 最近更新时间（取当前日期作为近似）
  var statUpdated = document.getElementById("stat-updated");
  if (statUpdated) {
    statUpdated.textContent = new Date().toISOString().split("T")[0];
  }

  // 运行天数
  var statUptime = document.getElementById("stat-uptime");
  if (statUptime) {
    var now = new Date();
    var diff = Math.floor((now - SITE_CREATED) / (1000 * 60 * 60 * 24));
    statUptime.textContent = diff + " 天";
  }
});
