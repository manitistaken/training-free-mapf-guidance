# Training-Free Online Guidance for Lifelong Multi-Agent Path Finding

Manit Agarwal · research in progress (October 2026)

In lifelong multi-agent path finding (MAPF), hundreds of robots share a grid map and receive a new
goal as soon as they reach the current one, so throughput collapses when crowds form. Guidance Graph
Optimization (GGO) addresses this by optimizing the planner's edge weights offline for a given map and
agent count. This project guides the PIBT planner **without training**: there is no per-map or
per-agent-count optimization, only one global parameter setting chosen by a small search on three maps.
Each robot reads two live signals computed from the current traffic, a smoothed map of where crowding
is building up and a trail left by recent movement, and uses them to choose its route and its next
move. Traffic rules such as one-way lanes can **emerge** from these signals instead of being designed
by hand. On the warehouse-33x36 map with 150 agents (3 seeds, earlier untuned trail settings), the movement trail alone made the side
aisles one-way: one-wayness 0.92–0.94 on a 0–1 scale, against 0.10–0.19 for plain PIBT and 0.88–0.96
for GGO's optimized graph (the crisscross baseline enforces this by design). The middle aisles stayed
two-way, and at 400 agents the effect is weaker (0.58–0.82).

## Results

Throughput (tasks completed per step), mean ± standard error over 5 seeds. Best in each row in bold.
These are 3 of the 9 PIBT settings for which GGO's repository provides optimized guidance graphs (8
appear in the GGO paper; warehouse-33x36 with 220 agents is in the repository only); random-32-32-20
and the other 5 are discussed below. All
rows use GGO's priority rule without the swap operation (see **Priority rule and swap** below).
GGO's graphs were optimized for exactly these (map, agent count) settings, but in GGO's own C++
simulator; their performance can change in my re-implementation, so this is not a reproduction of
the GGO paper.

| Map | Agents | PIBT (plain) | Crisscross | GGO (CMA-ES) | GGO (PIU) | Ours (training-free) | Ours vs plain PIBT | Ours vs best GGO |
|---|---|---|---|---|---|---|---|---|
| random-64-64-20 | 1500 | 5.64 ± 0.09 | 7.81 ± 0.16 | 9.17 ± 0.21 | 8.51 ± 0.30 | **9.87 ± 0.33** | +75% | +7.6% |
| room-64-64-8 | 1500 | 2.49 ± 0.02 | 2.65 ± 0.02 | **3.10 ± 0.03** | 3.08 ± 0.07 | 2.90 ± 0.01 | +17% | -6.6% |
| maze-32-32-4 | 400 | 1.12 ± 0.06 | 1.09 ± 0.05 | 1.22 ± 0.12 | **1.44 ± 0.06** | 1.18 ± 0.04 | +5% | -18.5% |

**random-32-32-20 is left out.** On this map my PIBT (GGO's priority rule, no swap operation, as in
GGO's code) scores far below the GGO paper: plain PIBT 1.52 vs 5.52 and GGO CMA-ES 1.82 vs 7.78. A second PIBT simulator I wrote separately (not using POGEMA's simulator) shows what happens with the
same rule: by the end of the episode 100% of agents are stuck (no net progress in the last 50 steps). Adding the swap operation removes most of this for plain PIBT
(10% stuck, throughput 5.41). It does not for GGO's graphs under GGO's rule (81% stuck with CMA-ES);
GGO CMA-ES reaches 7.05 only with swap and a different priority rule. GGO's own simulator also runs PIBT without swap, and the paper's throughput (5.52) shows no such
gridlock, so this is a difference in my implementation that I have not yet found. My method has not been evaluated without the gridlock, so I report no result on this map.
On the three maps in the table, my plain PIBT is within 7% of the paper's values.

The lead over GGO on random-64-64-20 holds in GGO's setting (GGO's priority rule, no swap; see
**Priority rule and swap** below), is about 1.8 standard errors of the difference with 5 seeds, and is
not yet confirmed with more seeds. On the other five settings (warehouse-33x36 with 150, 220 and
400 agents, empty-48-48 with 1000, den312d with 1200), where my method has so far been run only in
an earlier, untuned configuration, my method is 0.3% to 19.1% below the best GGO graph.

**Priority rule and swap.** All results above use GGO's priority rule and PIBT without the swap
operation, as in GGO's code. Both choices matter on random-64-64-20 with 1500 agents. In my second
simulator, plain PIBT in this setup also ends with 96% of agents stuck, as on random-32-32-20. Adding
swap alone cuts this to 12% and raises its throughput from 5.90 to 9.17; a priority rule that orders
agents by distance to the goal raises it to 8.44 even without swap. With that distance rule, GGO's
graphs reach 10.17 (CMA-ES) and 10.42 (PIU) without swap, and 10.58 (CMA-ES) and 10.76 (PIU) with
swap. These are second-simulator numbers, and my method has not been run there. They are all above my
method's 9.87 in the main simulator, and the two simulators agree closely in GGO's setting (plain PIBT
5.64 vs 5.90, GGO CMA-ES 9.17 vs 9.19, GGO PIU 8.51 vs 8.65). In the second simulator, plain PIBT's
throughput in GGO's setting is close to the GGO paper's (5.90 vs 6.01); I have not checked whether
GGO's simulator also gridlocks here. Most of the +75% over plain PIBT in the table could therefore also
be obtained by swap alone (+55% in the second simulator), and my method has
not yet been evaluated with swap or with the distance rule.

**Setup.** Lifelong MAPF: each agent gets a new goal when it reaches its current one, episodes of 1000
steps. All results in the table and the five other settings run in my own Python implementation of PIBT on
the POGEMA simulator, and GGO's
published guidance graphs are loaded unchanged into the same planner. Absolute numbers therefore
differ from the GGO paper. One parameter setting is used for every row of the table. It was tuned on separate seeds (100–102) of random-32-32-20 (400 agents, in the gridlocked setup described
above), maze-32-32-4
(400) and room-64-64-8 (500); random-64-64-20 was never used for tuning. Seeds 0–4 were also used
during development, so they are not a held-out test set; a held-out evaluation is planned. Per-seed
results are in [results/](results/).

## Code

The code is not public yet because a paper is in preparation. It is available on request for
academic review: [manit.agarwal21@gmail.com](mailto:manit.agarwal21@gmail.com).

**Code fingerprint.** The code and every result file behind this page are in my private repository
at commit

`9907e396e34a83414031e88b38c6522d65e646c8`

A commit ID is a cryptographic hash that covers every tracked file in the repository and its full history,
so changing any file would change it. The code at this commit
reproduces exactly 40 earlier episodes (random-32-32-20 with 200 and 400 agents), and the
15 short reference episodes in its test suite (recorded from the first snapshot of the
code). The table, the five other GGO settings and the lane measurement were produced in
early October 2026, shortly before the repository was put under version control. When I share the
repository, anyone can check the ID with `git rev-parse --verify <the ID above>^{commit}`.

## Credits

This work builds on the following. None of their code or data files are included in this repository.

- **PIBT.** K. Okumura, M. Machida, X. Défago, Y. Tamura. *Priority Inheritance with Backtracking for
  Iterative Multi-agent Path Finding.* IJCAI 2019; extended version in *Artificial Intelligence* 310,
  103752, 2022. The planner used here is my own Python re-implementation, written to follow the PIBT
  settings in GGO's code (its fork of WPPL: H. Jiang, Y. Zhang, R. Veerapaneni, J. Li, *Scaling Lifelong
  Multi-Agent Path Finding to More Realistic Settings: Research Challenges and Opportunities*, SoCS 2024),
  but not checked against it numerically.
- **GGO.** Y. Zhang, H. Jiang, V. Bhatt, S. Nikolaidis, J. Li. *Guidance Graph Optimization for Lifelong
  Multi-Agent Path Finding.* IJCAI 2024. [arXiv:2402.01446](https://arxiv.org/abs/2402.01446).
  Code: [lunjohnzhang/ggo_public](https://github.com/lunjohnzhang/ggo_public), commit `d8de695` (its
  README states the code is under the MIT License, except `RHCR/`, which I did not use). From that
  repository I used the map files, GGO's published optimized guidance graphs (CMA-ES and PIU) and the
  crisscross baseline graphs. For the results on this page only GGO's published files were used; GGO's
  optimizer, update models and C++ simulator were not run.
- **Swap operation** (used only in the second-simulator checks): from LaCAM2, the code of K. Okumura,
  *Improving LaCAM for Scalable Eventually Optimal Multi-Agent Pathfinding*, IJCAI 2023
  ([Kei18/lacam2](https://github.com/Kei18/lacam2), MIT License). My Python port follows its version in
  [DiligentPanda/MAPF-LRR2023](https://github.com/DiligentPanda/MAPF-LRR2023) (MIT License; Jiang et al.,
  SoCS 2024, cited above).
- **Crisscross** follows L. Cohen, *Efficient Bounded-Suboptimal Multi-Agent Path Finding and Motion
  Planning via Improvements to Focal Search*, PhD thesis, University of Southern California, 2020 (as
  defined in GGO).
- **Maps.** random-32-32-20, random-64-64-20, room-64-64-8, maze-32-32-4, empty-48-48 and den312d
  come from the MovingAI benchmarks (layouts identical to the MovingAI files; MovingAI data is under
  the Open Data Commons Attribution License): R. Stern et al., *Multi-Agent Pathfinding: Definitions,
  Variants, and Benchmarks*, SoCS 2019, and N. Sturtevant, *Benchmarks for Grid-Based Pathfinding*,
  IEEE Transactions on Computational Intelligence and AI in Games 4(2), 2012. den312d is a map from the
  game *Dragon Age: Origins* (BioWare), distributed through that benchmark.
- **Warehouse.** The warehouse-33x36 map and its task setup come from J. Li, A. Tinka, S. Kiesel,
  J. W. Durham, T. K. S. Kumar, S. Koenig, *Lifelong Multi-Agent Path Finding in Large-Scale
  Warehouses*, AAAI 2021, and Y. Zhang, M. C. Fontaine, V. Bhatt, S. Nikolaidis, J. Li, *Multi-Robot
  Coordination and Layout Design for Automated Warehousing*, IJCAI 2023; I used the version in GGO's
  repository.
- **POGEMA** (MIT License). A. Skrynnik, A. Andreychuk, A. Borzilov, A. Chernyavskiy, K. Yakovlev,
  A. Panov. *POGEMA: A Benchmark Platform for Cooperative Multi-Agent Pathfinding.* ICLR 2025.
  [Cognitive-AI-Systems/pogema](https://github.com/Cognitive-AI-Systems/pogema). Also
  [pogema-toolbox](https://github.com/Cognitive-AI-Systems/pogema-toolbox) (Apache License 2.0).
- **Ant colony optimization**, the inspiration for the movement-trail signal: M. Dorigo, V. Maniezzo,
  A. Colorni. *Ant System: Optimization by a Colony of Cooperating Agents.* IEEE Transactions on Systems,
  Man, and Cybernetics, Part B 26(1), 1996.
- **NumPy and SciPy** (permissive open-source licenses) for computation and shortest paths.
- **Related work** on online, traffic-adaptive guidance: H. Zang, Y. Zhang, H. Jiang, Z. Chen,
  D. Harabor, P. J. Stuckey, J. Li, *Online Guidance Graph Optimization for Lifelong Multi-Agent Path
  Finding*, AAAI 2025; Z. Chen, D. Harabor, J. Li, P. J. Stuckey, *Traffic Flow Optimisation for
  Lifelong Multi-Agent Path Finding*, AAAI 2024; T. Arita, K. Okumura, *Local Guidance for
  Configuration-Based Multi-Agent Pathfinding*, AAAI 2026; T. Arita, K. Okumura, *Lifelong LaCAM with
  Local Guidance for Lifelong MAPF*, SoCS 2026.
- **AI assistance.** The code, analysis scripts and this page were developed with the help of an AI
  coding assistant (Claude, by Anthropic).

## License

Copyright © 2026 Manit Agarwal. All rights reserved. See [LICENSE](LICENSE).
