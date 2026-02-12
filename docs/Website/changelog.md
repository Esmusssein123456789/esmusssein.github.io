---
changelog: True
---

# :material-history: 更新记录

!!! abstract "关于"
    这里记录了本站每次比较重要的内容更新。  
    小改动（如修正错别字、格式调整）不会单独列出。

---

{{ changelog }}

---

!!! tip "如何维护更新记录"
    本页由 [mkdocs-changelog-plugin](https://github.com/TonyCrane/mkdocs-changelog-plugin) 自动渲染，**但数据需要手动维护**。
    
    每次发布新笔记后，编辑项目根目录的 `changelog.yml`，在最新日期下添加条目即可：
    ```yaml
    - "changelog":
      - "2026-03-15":
        - "newpage":
            text: "页面名称"
            href: /板块/页面/
        - "pageupdate": 更新了某某内容
    ```
    
    详细说明请参考 [操作流程 > 更新记录](workflow.md#6)。
