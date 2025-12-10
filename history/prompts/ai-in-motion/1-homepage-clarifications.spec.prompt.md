---
id: 1
title: Homepage Clarifications
stage: spec
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: ai-in-motion
branch: main
user: FRIENDX COMPUTERS
command: /sp.clarify
labels: [homepage,ui,clarification]
links:
  spec: specs/ai-in-motion/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/ai-in-motion/spec.md
tests:
  - null
---

## Prompt

sp.specify : Generate a complete Docusaurus homepage for the project
"AI in Motion — Foundations of Physical AI and Humanoid Robotics" with the following specifications:

------------------------------------------------
🔵 HERO SECTION
------------------------------------------------
- Large, bold title: "AI in Motion"
- Subtitle: "Foundations of Physical AI and Humanoid Robotics"
- Center-aligned
- Two buttons:
  1. "Start Learning" → primary button
  2. "Open Book" → outline button
- Background gradient: #e3f2fd → #f8fbff
- Soft shadow for buttons, rounded corners
- Large padding top & bottom

------------------------------------------------
🔵 FEATURE SECTION
------------------------------------------------
- Below hero, add 3 feature cards with icons:
  1. Physical AI Basics → CPU/Brain icon → "Introduction to embodied intelligence fundamentals."
  2. Humanoid Robotics → Robot icon → "Learn structure and control of humanoid robots."
  3. Motion Intelligence → Motion graph icon → "Explore movement, balance, and real-world physics."
- Cards: white background, rounded corners (18px), soft shadow, hover lift effect
- Responsive: 3-column desktop → 1-column mobile

------------------------------------------------
🔵 GLOBAL URDU TRANSLATE BUTTON
------------------------------------------------
- Navbar top-right: "Translate to Urdu" button
- Button triggers AI translation for **entire page content**, including tutorials, blog posts
- Original English content remains intact
- Add helper JS function: src/utils/translate.js
- Styling: consistent with site theme, rounded, soft shadow, hover effect

------------------------------------------------
🔵 FILES TO GENERATE
------------------------------------------------
1. src/pages/index.js → Homepage with hero + feature section
2. src/components/HomepageFeatures/index.js → Feature cards component
3. src/components/TranslateButton/index.js → Global Translate button component
4. src/utils/translate.js → AI helper function for page translation
5. src/css/custom.css → Styling for hero, buttons, features, and translate button

Use this format exactly:

---file:path---
<content>

## Response snapshot

I analyzed the existing specification and identified key ambiguities related to the homepage requirements that weren't detailed in the original spec. I asked three clarifying questions and received responses that will guide the implementation:

1. Open Book button functionality: Link to main curriculum documentation
2. Feature card icons: Use emoji icons as specified
3. Urdu translation behavior: Toggle between English and Urdu on button click

I've updated the specification file to incorporate these clarifications in the clarifications section and added detailed requirements to the Documentation Platform section.

## Outcome

- ✅ Impact: Clarified homepage UI requirements for Docusaurus implementation
- 🧪 Tests: null
- 📁 Files: specs/ai-in-motion/spec.md
- 🔁 Next prompts: /sp.plan for implementation planning
- 🧠 Reflection: Critical ambiguities resolved that would impact UI implementation

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): Proceed to planning phase