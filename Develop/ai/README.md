AI Module
=========

Handles recognition, analysis, decision-making, and AI Agent orchestration
for the Autonomous AI Robot.


Directory Layout
----------------

    ai/
        agents/
            agents.py               Agent definitions (Converter, Verifier, Reviewer)
            tasks.py                Task definitions for each agent
            tools.py                Custom tools (syntax check, logic analysis)
            main.py                 Entry point to run the pipeline

        perception/
            object_detection.py     Detect and classify objects in camera frames
            depth_estimation.py     Estimate distance using Depth Anything V2
            camera_interface.py     Camera connection (IMOU RTSP, USB)

        planning/
            path_planner.py         Path finding algorithms
            decision_engine.py      Decision-making for robot actions

        control/
            motion_controller.py    Convert AI decisions to hardware commands


Technology
----------

    Python 3.10+
    CrewAI                  Multi-Agent orchestration
    OpenCV                  Image processing
    Depth Anything V2       Monocular depth estimation
    OpenAI / Azure OpenAI   LLM backend for Agents
