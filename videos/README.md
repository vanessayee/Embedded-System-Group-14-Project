# Videos

Short videos of tests and demos running on the real hardware. A video shows
what a log cannot: the robot following the line, turning at a barcode
junction, going around an obstacle and rejoining the line. List **every** video in the table at the bottom of this page.

## What to record

- **Short:** 15–60 seconds. One test or one feature per video.
- **Say what it is.** Show or say the test ID (e.g. `S2-U03`, `INT-01`) or
  the demo name at the start.
- **Show the commit.** Before recording, run `git rev-parse --short HEAD` and
  keep the terminal in shot for a moment, or put it in the table below.
- **Show the result.** Keep both the hardware and, if it matters, the
  console output in view.
- **Landscape**, steady, good light. No music needed.

## How to share it: unlisted YouTube only

1. Upload the video to YouTube with visibility set to **Unlisted**: anyone
   with the link can watch it, but it does not show up in search. Not
   *Private* (only invited accounts can watch, so the link won't work for
   your team or instructor), and not *Public*.
2. Copy the video's link (e.g. `https://youtu.be/...`).

**Do not commit video files to this repo or attach them on GitHub.** A video
in the repo stays in the git history for good, even if deleted later, and
slows every clone for the whole team. Video files are ignored by
`.gitignore` for this reason.

Then add a row to the table below. If the video is evidence for a test, also
put its link in the *Log / evidence* column of that test's row in
`testing/.../TEST_LOG.md`.

## Video index

Newest at the bottom. Never delete a row.

Subsystem lists the participating buddies. Test IDs are matched to the related cases in `testing/`; commit hashes were not provided.

| Date | Subsystem | Test ID / demo | Commit | Student ID | Link | Notes |
|---|---|---|---|---|---|---|
| 2026-11-24 | Buddy 2 and 3 | S2-U09, S2-U23, S2-U24, S2-U25, S3-U22, S3-U23, S3-U24, S3-U25, S3-U26, S3-U27, S3-U32 | a8bd0cb4c79e13a357bdeb51c7f466c39c40f836 | 2502591, 2501720 | [Watch](https://youtube.com/shorts/hkbLHrSwU24?feature=share) | Line following |
| 2026-11-24 | Buddy 2 and 4 | S2-U09, S4-U19, S4-U20, S4-U21, S4-U22, S4-U23, S4-U24, S4-U25, S4-U42, S4-U43, S4-U55, S4-U58, S4-U60, S4-U64 | a8bd0cb4c79e13a357bdeb51c7f466c39c40f836 | 2502591, 2500320 | [Watch](https://youtube.com/shorts/3ayGtST2gXk?feature=share) | Hump detection |
| 2026-11-24 | Buddy 2 and 5 | S2-U09a, S2-U09b, INT-10a, INT-10b, INT-14a, INT-14b, INT-14c, INT-14d, S5-U14, S5-U15, S5-U15b, S5-U16, S5-U16b, S5-U16c, S5-U17, S5-U17b, S5-U17c, S5-U17d, S5-U18, S5-U18a, S5-U19, S5-U20 | a8bd0cb4c79e13a357bdeb51c7f466c39c40f836 | 2502591, 2502075 | [Watch](https://youtube.com/shorts/WG28bTAuabs?feature=share) | Obstacle avoidance |
| 2026-11-24 | Buddy 1, 2 and 3 | S1-U09m, S2-U10c, S3-U15, S3-U17 | a8bd0cb4c79e13a357bdeb51c7f466c39c40f836 | 2501573, 2502591, 2501720 | [Watch](https://youtube.com/shorts/WsfCsnMzaSI?feature=share) | Barcode detection (turn left) |
| 2026-11-24 | Buddy 1, 2 and 3 | S1-U09m, S2-U10b, S3-U15, S3-U17 | a8bd0cb4c79e13a357bdeb51c7f466c39c40f836 | 2501573, 2502591, 2501720 | [Watch](https://youtube.com/shorts/GAn-FjgTnMk?feature=share) | Barcode detection (turn right) |
| 2026-11-24 | Buddy 1, 2 and 3 | S1-U09m, S2-U10g, S3-U15, S3-U17 | a8bd0cb4c79e13a357bdeb51c7f466c39c40f836 | 2501573, 2502591, 2501720 | [Watch](https://youtube.com/shorts/QHz527VCt4E?feature=share) | Barcode detection (U-turn) |















