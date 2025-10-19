# LLM-Interface-for-Robot-Manipulator-Control
Integration of Large Language Models (OpenAI GPT &amp; Mistral) with ROS-based robot manipulators to enable natural language control. Includes LLM–ROS connector, fine-tuned few-shot models, and path planning modules tested on Panda Arm in Gazebo simulation.

## Highlights & contributions

- A robust **LLM→ROS bridge** that parses natural language and outputs structured function calls (JSON) for manipulator execution.
- A `move_group` function schema (x,y,z,roll,pitch,yaw + `is_relative` flags) that standardizes execution commands.
- Fine-tuning/few-shot strategy for GPT-3.5 turbo to reliably extract numeric parameters and relative/absolute intent (41 training samples used for fine-tuning).
- Pattern path generator (square/triangle/circle) that expands high-level pattern instructions into sequential subtasks and executes them.
- Tested in simulation (Panda — MoveIt! + Gazebo). Training loss reported: **0.1206** (MSE).

### 🎬 Live Demonstration
**Download / play video:**  
[Simulation video (MP4)](https://github.com/Preeti2802/LLM-Interface-for-Robot-Manipulator-Control/raw/main/llm_panda_simulation.mp4)


