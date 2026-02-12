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
    编辑 `docs/changelog.yml` 文件，按如下格式添加条目即可，插件会自动生成时间线：
    ```yaml
    - "changelog":
      - "2026-03-15":
        - "newpage":
            text: "页面名称"
            href: /板块/页面/
        - "pageupdate": 更新了某某内容
        - "feature": 新增了某某功能
    ```
    
    可用的更新类型：
    
    | 类型 | 说明 |
    | :--- | :--- |
    | `newpage` | 新增页面 |
    | `pageupdate` | 页面更新 |
    | `feature` | 功能性更新 |
