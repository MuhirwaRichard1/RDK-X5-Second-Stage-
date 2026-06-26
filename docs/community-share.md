# Community Share (Deliverable #5)

Post this summary to the program community channel, then put the **permalink** to your post in your
submission PR file under `projects/`.

---

## Suggested post

> **🤖 RDK X5 Tri-Cam NavBot — vision-only indoor navigation, no wheel encoders**
>
> Building an indoor robot on the **RDK X5** that navigates and localizes using **3 USB cameras only**
> (front HBVCAM OV2710 100° + 2 wide-angle sides) — no encoders, no GPS.
>
> - **BPU:** YOLO11 detection, already running at **~10 FPS** on-device
> - **CPU (8× A55):** monocular **visual odometry** + 3-cam surround free-space + open-loop L298N drive
> - **Twist:** encoderless — we close the motor loop **visually** via a measured duty↔velocity table,
>   not wheel ticks. Every module has a defined failure mode and a numeric risk-pivot trigger.
>
> Target: ≤150 ms sensor→motor latency, ≤5% VO drift on a 10 m loop, ≥8/10 mission success.
>
> Full proposal (Challenge 1–3), architecture diagrams, BOM and 7-week roadmap:
> 👉 **<REPO_URL>**
>
> Feedback welcome — especially on encoderless localization! #RDKBuilder #RDKX5

---

## Checklist before submitting
- [ ] Repo is **public**
- [ ] `PROPOSAL.md`, `ROADMAP.md`, `docs/bom.md` render correctly on GitHub (Mermaid included)
- [ ] GitHub Projects board (or ROADMAP milestones) is public and linked in README
- [ ] Community post published; **permalink** copied
- [ ] PR opened to the program repo with a showcase file under `projects/` (per `projects/README.md`)
- [ ] Showcase file contains: repo link + community-post permalink
