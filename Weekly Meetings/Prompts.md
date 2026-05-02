
You are working on a research-grade robotics problem. 

Requirements:
- Be technically rigorous and precise
- Avoid generic explanations
- Always reason at the level of system equations, observability, or algorithm structure where applicable
- Prefer depth over breadth
- Keep outputs structured and concise

---

Task 1:  

I am working on improving the performance of LiDAR-inertial odometry using FAST-LIO2. FAST-LIO2 builds on FAST-LIO to make an IEKF-based LiDAR-inertial odometry framework. I am using a specific sensor rig: I am using a Livox mid-360 LiDAR and its integrated IMU. This is a solid-state LiDAR with a 360° * 59° FOV. My main task was to look at some datasets where FAST-LIO2 failed from my research company and ascertain why FAST-LIO2 failed, a.k.a. what were its failure modes, and then to improve FAST-LIO2 to tackle these failure modes and improve its robustness.  
  
Look at my meeting notes and see what types of problems these failure modes could have been for yourself. Try to ascertain all possible issues and list them, and explain what they all are in detail. Explain which dataset you were refering to, or if there are additional failure modes that you think are important to tackle you may also refer to them here. Do this by writing a markdown document called `failure_modes.md` explaining all of these failure modes in detail. Things to look into: geometric degeneracy, attitude mis-estimation, lack of features...  

Your output should be a markdown file called `failure_modes.md`.

---

Task 2:  

Sub-task a:

I am working on improving the performance of LiDAR-inertial odometry using FAST-LIO2. FAST-LIO2 builds on FAST-LIO to make an IEKF-based LiDAR-inertial odometry framework. I am using a specific sensor rig: I am using a Livox mid-360 LiDAR and its integrated IMU. This is a solid-state LiDAR with a 360° * 59° FOV. My main task was to look at some datasets where FAST-LIO2 failed from my research company and ascertain why FAST-LIO2 failed, a.k.a. what were its failure modes, and then to improve FAST-LIO2 to tackle these failure modes and improve its robustness.  

Context:  
- All failure modes that have been looked into are included in `failure_modes.md`. You may use this as the ground truth for failure modes.
- I have already collected a set of papers, which are included in the system files you have access to.  

High-level explanation of what you need to do:
- Look at all of the attached papers, which are the papers that I have found so far that can be useful to attack these failure modes.   
- Now that you know what papers I have already gathered, look yourself into papers that could be useful for all possible failure modes, including   
     - the failure modes that we have already looked into  
     - failure modes that I may not have know beforehand which you have ascertained that could be a source of our failure  
  
More specifically, your tasks are:  
  
1) Go through the attached papers and identify:  
- What method each paper proposes  
- Which failure mode(s) from `failure_modes.md` it addresses  
  
2) Identify missing coverage:  
- Which failure modes are NOT well addressed by the current papers  
  
3) Search for additional relevant papers to cover:  
- uncovered failure modes  
- weakly addressed failure modes  
  
1) Produce a structured mapping document called `papers_map.md` with the following format:  
  
# Failure Mode: name  
  
## Existing Papers  
- Paper Name  
- Core idea (2–3 lines)  
- Strengths  
- Limitations (especially w.r.t IEKF and solid-state LiDAR)  
  
## Missing Gaps  
- What is still not solved  
  
## Additional Suggested Papers  
- Paper Name  
- Why it is relevant  
- Which gap it fills  
  
Requirements:  
- Be concise and structured  
- Do NOT explain papers in full detail yet  
- Focus on mapping, not deep explanation


---

Sub-task b:  

Context:  
- Use `papers_map.md` as the guide for which papers to analyze.

Your task:
- For EACH paper listed in `papers_map.md`, create a separate markdown file:  
`papers/<paper_name>.md`. In each of these, explain the important parts of each paper in detail, so that I don't have to read them all individually. Include all important aspects, including detailed explanations, equations, etc., everything I need to know to fully understand the papers. Important, for each paper:  
- mention how applicable they are to an IEKF-based system (specifically FAST-LIO2), and also for solid-state LiDARs like the Livox mid-360  

Do NOT summarize superficially. Focus on:
- key equations
- assumptions
- limitations

---

Sub-task c:  

Context:  
- Failure modes are defined in `failure_modes.md`  
- Detailed paper notes are in `papers/*.md`

Your task:  
  
Create a master summary document called `papers_summary.md`.
- Make a main summarized markdown document which goes over all of the papers in a summarized fashion. Here, the most important part is to make it clear what each method does, what failure mode each method tackles, such that I can look into them more quickly. If I want more detail, I will look at the other .md documents which cover them in detail. You should like those .md documents in this one (use obsidian linking method)  
  
You must  
1) Clearly separate papers (or sub-parts of papers) according to which failure modes they tackle  
2) Explain which methods are most applicable for my IEKF-based system (specifically FAST-LIO2), and also for my solid-state LiDARs (Livox mid-360)  

---

Task 3:  
  
Sub-task a:  

I am working on improving the performance of LiDAR-inertial odometry using FAST-LIO2. FAST-LIO2 builds on FAST-LIO to make an IEKF-based LiDAR-inertial odometry framework. I am using a specific sensor rig: I am using a Livox mid-360 LiDAR and its integrated IMU. This is a solid-state LiDAR with a 360° * 59° FOV. My main task was to look at some datasets where FAST-LIO2 failed from my research company and ascertain why FAST-LIO2 failed, a.k.a. what were its failure modes, and then to improve FAST-LIO2 to tackle these failure modes and improve its robustness.  

- Given all of the information we have so far, including the failure modes of FAST-LIO2 (see `failure_modes.md`) and what methods in the literature there are that tackle these problems (see `papers_summary.md`), I need you to come up with novel solutions that have not been tackled in the literature yet as to how to tackle our failure modes. Specifically  
	- these must be applicable to an IEKF system and solid-state LiDARs  
	- the novelty may just be how to modify an existing method to be implemented to an IEKF system or solid-state LIDARs. It may also be a completely novel method outright.   
Remember, the method must be  
- implementable on FAST-LIO2  
- have good potential upside  
Use information from all papers we have looked at so far to guide you.  

Write all of the proposed methods down in a new markdown file called `proposed_methods.md`.
For each proposed method:

Describe how this integrates into the IEKF or FAST-LIO2 pipeline.
Be explicit about:
- state update modifications
- measurement model changes
- covariance handling

Avoid proposing ideas that require:
- full system redesign
- non-real-time computation
- training-heavy pipelines (unless justified)

---

Sub-task b:  
  
- Given all of the information we have so far, including the failure modes of FAST-LIO2 (see `failure_modes.md`), what methods in the literature there are that tackle these problems (see `papers_summary.md`), and the novel methods you proposed (see `proposed_methods.md`), I need you to try to come up with the optimal plan to improve FAST-LIO to deal with our failure modes. Specifically  
     - If these methods are not directly applicable to an IEKF-system or solid-state LiDAR, explain how they should be modified such that they can work with these systems. Do this in detail.  
     - Explain which methods are useful for what failure modes  
     - These methods need to work well together  
     - Explain which methods will have the most impact (rank them), and how difficult they will be to implement  
     - Don't make an unreasonable list of methods. Make it realistic. It may include multiple new additions, that is ok, but it shouldn't be everything imaginable. Make it optimal, realistic, with the highest chance to improve our FAST-LIO2 performance. And bonus points go to if it becomes a scientific publication worthy method!

Your output should be a new markdown file called `improvement_plan.md`.