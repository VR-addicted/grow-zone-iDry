# iDRY-26 Future Roadmap & TODO List 📝

This document tracks planned future features, Home Assistant integrations, and architectural enhancements for upcoming sessions.

---

## 🔮 Planned Future Features & Enhancements

### 1. Home Assistant Deep Integration & Remote Controls
- **MQTT Command Topics & Controls:**
  - Remote control of `dry_strategy` (60/60, VPD, VPD AUTO) via HA select entity.
  - Remote adjustment of Hygro-Limit (70%, 75%, 80%).
  - Remote adjustment of Stoßlüftungstimer (`purge_interval_min` & `purge_duration_sec`).
- **Custom Home Assistant Dashboard Panel:**
  - Build a custom Lovelace card / dashboard matching the iDRY-26 dark blue aesthetic with live Sanduhr & Rotor moon graphics.

### 2. Security & Network Hardening (Optional End-Game Milestone)
- **HTTPS / TLS Encryption:** Secure HTTPS WebServer endpoints.
- **MQTT TLS Client Certificates / Encrypted Payload:** Secure broker authentication with certificates.

### 3. Obsidian Knowledge Base Connection
- **Antigravity <-> Obsidian Vault Integration:**
  - Direct markdown reading/writing of Obsidian Vault notes.
  - Optional MCP (Model Context Protocol) Obsidian server integration (`mcp-server-obsidian`) to sync project logs, technical docs, and growth journals directly into Obsidian notes.

---

*Document created for iDRY-26 joint pair-programming roadmap.*
