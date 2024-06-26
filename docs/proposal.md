# Proposal

**Project Name**: End-to-end autonomous vehicle driving based on text-based instructions.

## Abstract
JdeRobot is an open-source toolkit for developing Robotics applications. Amongst various projects in Behavior Metrics for evaluating DL models' performance in autonomous driving tasks. In this project, the aim is to integrate a Language Model (LM) system with an end-to-end autonomous driving model. By combining previous knowledge and successful projects, the goal is to enable users to provide text-based commands directly to the vehicle, similar to interacting with a real-life taxi. The project will commence with a focus on simplicity, utilizing models like BERT, and gradually iterating towards more complex architectures.

```{figure} _static/BERT-open-loop-light.svg
:align: center
:figwidth: 90%
:figclass: only-light

Integrating BERT or a similar Language Model (LM) for generating High-Level Commands (HLC), to perform autonomous driving using Carla simulation.
```

```{figure} _static/BERT-open-loop-dark.svg
:align: center
:figwidth: 90%
:figclass: only-dark

Integrating BERT or a similar Language Model (LM) for generating High-Level Commands (HLC), to perform autonomous driving using Carla simulation.
```

```{figure} _static/BERT-closed-loop-light.svg
:align: center
:figwidth: 90%
:figclass: only-light

Using Vision Models such a LLaVA for providing feedback to language model.
```

```{figure} _static/BERT-closed-loop-dark.svg
:align: center
:figwidth: 90%
:figclass: only-dark

Using Vision Models such a LLaVA for providing feedback to language model.
```

## Timeline

| **Time**                              | **Tasks**                                                                                                                                                                                                                                                                                                                                                                     |
|---------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| Community Bonding Period              | • Thoroughly familiarize myself with the code base.<br>• Set up a blog website for project documentation and updates.<br>• Conduct a comprehensive literature survey to identify relevant LM architectures and fine-tuning techniques.<br>• Discuss project groundwork and implementation strategies with mentors.                                                            |
| Week 1, 2 & 3<br>(May 27 - Jun 17)    | • Implement a basic NLP-based controller using BERT or a similar LM.<br>• Develop an initial prototype for text-based command input and vehicle control.<br>• Finetuning and training BERT model for generating HighLevelCommands(HLCs).<br>• This will update [get_random_hlc](https://github.com/TheRoboticsClub/gsoc2023-Meiqi_Zhao/blob/676df573082c0826dba65fe34755cf70767aa144/src/utils/high_level_command.py#L35) and instead of randomized HLC, text-based HLCs will be produced.                                |
| Week 4 & 5<br>(Jun 17 - 30)           | • Understanding and integrating vision encoders (like [LMDrive](https://github.com/opendilab/LMDrive)) for closed-loop control. (The above is open-loop setting)<br>• Study and discuss the feasibility of reproducing other approaches like [Driving-with-LLMs](https://github.com/wayveai/Driving-with-LLMs) within the project framework.                                                                                                                          |
| Evaluation Week 6 & 7<br>(Jul 1 - 12) | • Train the integrated system on the LMDrive [dataset](https://huggingface.co/datasets/OpenDILabCommunity/LMDrive) for performance evaluation.<br>• Explore the possibility of creating a custom dataset using [data_collector.py](https://github.com/TheRoboticsClub/gsoc2023-Meiqi_Zhao/blob/main/src/data_collector.py) to further enhance training data diversity and model robustness.<br>• Meet the Phase 1 Evaluation deadline.                                                                                                |
| Jul 12                                | Phase 1 Evaluation deadline                                                                                                                                                                                                                                                                                                                                                   |
| Week 8 & 9<br>(Jul 15 - 29)           | • Explore the use of Vision-Language Models (VLMs) like [LLaVA](https://llava-vl.github.io/) to improve the system's understanding of visual inputs.                                                                            |
| Week 10 & 11<br>(Jul 29 - Aug 12)     | • Investigate extending the evaluation metrics using Visual Question Answering (VQA) techniques, such as [LingoQA](https://github.com/wayveai/LingoQA), to enhance system comprehension and response accuracy.                                                                                                                                                                                                                                                                                                                                                                         |
| Week 12 & 13<br>(Aug 12 - 26)         | • Finalize project deliverables, including code, documentation, and any additional materials.<br>• Conduct thorough testing and validation of the integrated system to ensure reliability and performance consistency.<br>• Prepare the final report summarizing project outcomes, challenges faced, solutions implemented, and future directions for potential improvements. |
| Week 14 & 15                          | • Buffer period for any unexpected delays or additional tasks.<br>• Finalize project deliverables and ensure all code and documentation are properly organized and submitted.                                                                                                                                                                                                 |

## References
* [ROSGPT_Vision: Commanding Robots Using Only Language Models' Prompts](https://arxiv.org/pdf/2308.11236.pdf)
* [LMDrive: Closed-Loop End-to-End Driving with Large Language Models](https://github.com/opendilab/LMDrive)
* [Visual Language Maps for Robot Navigation](https://vlmaps.github.io/)
* [Driving with LLMs: Fusing Object-Level Vector Modality for Explainable Autonomous Driving](https://github.com/wayveai/Driving-with-LLMs)