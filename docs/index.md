---
tocdepth: 4
---

# JdeRobot GSoC Proposal

```{toctree}
---
maxdepth: 2
caption: Contents
hidden:
---

Home <self>
Proposal <proposal>
Python <python>
C++ <cpp>
ROS2 <ros2>
RoboticsAcademy <ra>
```

```{admonition} PDF Format
:class: help
View this proposal in pdf format.<br>
Link: [GSoC Proposal (JdeRobot)](https://drive.google.com/file/d/1_RES6fp567Cf8mLL0pn3IAF2XBkKCAnE/view?usp=sharing)
```

## Introduction

Hi! <img src="_static/Hi.gif" height="1.5em">

```{figure} _static/abd.jpg
:align: right
:figwidth: 39%
:figclass: sidefigure

A candid moment captured at IvLabs, running "Self-Balancing Bicopter" project the last time before dismantling it to salvage its BLDCs. 
```

My name is Muhammed Abdullah Shaikh. I recently graduated with a degree in Electrical and Electronics
from [Visvesvaraya National Institute of Technology, Nagpur](https://en.wikipedia.org/wiki/Visvesvaraya_National_Institute_of_Technology_Nagpur), an institute of national importance.
For the past few months, my work has focused on embedded systems, particularly overseeing the network module supporting TCP, MQTT, and other activities such as FOTA for an NXP i.MX RT microcontroller via a GSM modem. 


```{figure} _static/turtlebot.gif
:align: left
:figwidth: 26%
:figclass: sidefigure

Managed to teleoperate the Kubuki Turtlebot 2 after a tough day of fixing issues.
```

Most of my collegiate work took place at [IvLabs, the AI and Robotics lab](https://www.ivlabs.in/) of VNIT. This is where I delved into one-shot and few-shot learning, computer vision and robotics.Notable projects include [Face-Unlock](https://github.com/IvLabs/Face-Unlock/tree/main), [PID controller](https://github.com/ABD-01/ros_pid), teleoperation of turtle bot, [Object Detection](https://github.com/IvLabs/Object-Detection), [Siamese Neural Nets](https://github.com/ABD-01/Siamese-NN), etc. I even made [contributions](https://github.com/pytorch/vision/pulls?q=is%3Aclosed+is%3Apr+author%3AABD-01) to PyTorch's Vision library. 


I am also proud to mention that my research endeavors have led to the acceptance of a paper titled "[Open Set Multi Source Multi Target Domain Adaptation](https://ivlabs.github.io/os-nsmt/)," at the Neurips Pre-Registration Workshop 2019. Later, I worked under the guidance of [Prof. Konda Reddy](https://krmopuri.github.io/) from [IIT Guwahati](https://en.wikipedia.org/wiki/IIT_Guwahati) on Dataset Condensation and [Coreset Extraction techniques](https://github.com/ABD-01/Coreset).


```{admonition} TL;DR
:class: tip
Electrical engineering graduate with a focus on robotics and embedded systems. Previosuly Vice-Chairmain at IvLabs, contributed in PyTorch Vision, and
 presented research work at NeurIPS Workshop. 
```

## Studies

**Would your application contribute to your ongoing studies/degree? If so, how?**

I am not currently enrolled at any university.

But if you were to ask "if this would contribute to future studies/degree?"

Then definitely Yes!

I plan to pursue Master's degree in domains of Robotics and Deep Learning. I am planning to apply for the Fall '25 term.
Having GSoC experience with JdeRobot would doubtlessly help my application. 

## Contact Details:
   * First Name: Muhammed Abdullah 
   * Surname: Shaikh
   * Country: India
   * Email: `muhammed.abd.shaikh AT gmail.com`
   * Public repositories: [github.com/ABD-01](https://github.com/ABD-01?tab=repositories)
   * Blogs:
      * [Creating GUI using PySide6](https://phase-wool-a41.notion.site/PySide6-651af89ca9cc407c8f74b120a2d0215e)
      * [Notes on Flask, WTForms, Protocol Buffers and more…](https://abd-01.github.io/Flask-Protobuf/Notes.html)
   * Links:
      * Website: [abd-01.github.io](https://abd-01.github.io/)
      * Github: [ABD-01](https://github.com/ABD-01)
      * Twitter: [_m_ABD](https://twitter.com/_m_ABD)
      * LinkedIn: [muhammedabd](https://www.linkedin.com/in/muhammedabd/)

## Programming Background
I owe a significant part of my programming journey to key mentors, including [Bucky Roberts](https://github.com/buckyroberts) (YouTube: [thenewboston](https://www.youtube.com/thenewboston)), [Prof. David J Malan](https://cs.harvard.edu/malan/) (Course: [CS50x](https://cs50.harvard.edu/x/2020/), [Certificate](https://certificates.cs50.io/3a1e1887-2383-40f3-b442-dd914bfe7f12.pdf)), and [Corey Schafer](https://www.youtube.com/channel/UCCezIgC97PvUuR4_gbFUs5g).

### <img src="_static/python.png" height="1em"> Python
In the realm of Python programming, I consider myself between an intermediate and advanced level.
While some measure expertise by counting lines of code, I've never kept a record. My Python projects span diverse domains, from *web scraping*, *data analysis*, and *image processing* to *deep learning*. I've crafted production-grade **GUI tools** for vendors, developed **TCP servers**, created dynamic websites with **Flask**, implemented **function decorators**, explored perfplots, delved into meta-programming (Python scripts generating JS functions), and wrote **Sphinx** based documentation, among various other endeavors.

My open-source contributions too were in Python. Checkout my [Python course certificate](https://github.com/ABD-01/Python-for-Everybody)

### <img src="_static/c-programmin.png" height="1.2em"> C
While Python has dominated my programming landscape, C remains my first love and the cornerstone of my coding journey. 
The bulk of my bread and butter stems from C programming. 
My academic courses `CSL101` and `CSL210` were also focused on C language. In addition, I've also worked with associated tools, such as makefiles and CMakeLists. 

### <img src="_static/Matlab_Logo.png" height="1em"> MATLAB
I have a degree in Electrical Engineering. Do you think one can escape the sleepless night of assignments and simulations in MATLAB?

## GSoC participation

**Have you participated in GSoC before?**

Nope

**Have you applied but were not selected? When?**

Yes! Twice! Fingers crossed for the 3rd time.

* For the first time in GSoC 2021, the mentor helped a lot in my proposal. I thank him for his time, motivated to keep applying next years. Applied to [INCF](https://www.incf.org/).

* Then next time applied to [OpenVINO](https://docs.openvino.ai/2023.3/home.html) and [Open3D](https://www.open3d.org/), completed their assignments ([Face-Verification](https://github.com/openvinotoolkit/openvino_notebooks/pull/509) and [Sparse-Convolution](https://github.com/ABD-01/SparseConv) respectively), and drafted decent proposals. 

* Organization said don't apply coz we already have lot's of competition.

**Have you submitted/will you submit another proposal for GSoC 2024 to a different org?**

I don't think so I will. 

This is the only proposal I have worked on so far.


## Feedback

Please do share your feedback.

```{raw} html

<style>
    form {
        padding: 25px;
        margin: 25px;
    }

    input,
    textarea {
        width: 90%;
        padding: 8px;
        margin-bottom: 20px;
        border: 1px solid #1c87c9;
        outline: none;
        color: var(--color-content-foreground);
        background-color: var(--color-admonition-title-background--admonition-todo);
    }

    button {
        width: 30%;
        padding: 10px;
        border: none;
        background: var(--color-admonition-title-background--seealso);
        font-size: 16px;
        font-weight: 400;
        color: var(--color-admonition-title--seealso);
        ;
    }

    button:hover {
        background: #2371a0;
    }

    @media (min-width: 568px) {
        .main-block {
            flex-direction: row;
        }
    }
</style>
<form id="ContactForm" onsubmit="event.preventDefault();">
    <div class="info">
        <input class="fname" type="text" name="name" placeholder="Name">
        <input type="text" name="email" placeholder="Email">
    </div>
    <p>Message</p>
    <div>
        <textarea name="message" rows="4" style="width: 90%;"></textarea>
    </div>
    <button id="submitButton">Submit</button>
</form>

<script>
var submitMessage = document.getElementById("submitButton"),
    ContactForm = document.getElementById("ContactForm");

function submit(){
    var url = "https://discord.com/api/webhooks/1205925744660971580/y5by-FiA8G058BiGApiSjZb1enCXGMnkTmIq_dAaGXxg6LXFAz6FV2qNbtcWahk4DApA";

    var xhr = new XMLHttpRequest();
    xhr.open("POST", url);

    xhr.setRequestHeader("Accept", "application/json");
    xhr.setRequestHeader("Content-Type", "application/json");

    xhr.onreadystatechange = function () {
       if (xhr.readyState === 4) {
          console.log(xhr.status);
          console.log(xhr.responseText);
       }};

    var data = {
      "content": "<@701479951479865384>, you have a new feedback on GSoC Proposal!",
      "embeds": [
        {
          "title": ContactForm.name.value,
          "description": "**Email**:" + ContactForm.email.value + "\n**Message**:" + ContactForm.message.value,
          "color": 22963
        }
      ]
    };
    xhr.send(JSON.stringify(data));
}

submitMessage.addEventListener('click',()=>{
    submit();
    alert("Message Sent")
})
</script>

```