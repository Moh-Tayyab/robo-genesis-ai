# Physical AI & Humanoid Robotics — Project Overview

## 📘 Project Title
**Physical AI & Humanoid Robotics: A Modern AI-Native Technical Textbook (RoboGenesis AI)**

This project is a complete AI-native textbook built using **Docusaurus**, **Spec‑Kit Plus**, and **Claude Code**, designed to teach the future of robotics: humanoids, embodied intelligence, simulation systems, ROS 2 control, NVIDIA Isaac AI, and Vision‑Language‑Action robotics.

This book also integrates **AI Agents**, **BetterAuth signup/signin**, **Personalized Content**, **Urdu translation**, and a full **RAG chatbot** built on Neon + Qdrant + FastAPI.

---

## 🎯 Project Goals
1. Teach *Physical AI* — AI that operates in the real world.
2. Provide a complete learning pathway across:
   - ROS 2  
   - Gazebo & Unity simulation  
   - NVIDIA Isaac Sim & Isaac ROS  
   - Vision-Language-Action Robotics  
   - Humanoid robot control  
3. Build a modern AI-native book using:
   - **Docusaurus 3**
   - **Spec‑Kit Plus**
   - **Claude Code Agents & Skills**
4. Ship an advanced **RAG search + chatbot** inside the book.
5. Personalize learning through:
   - **BetterAuth user onboarding**
   - **Dynamic personalization**
   - **Urdu translation toggle**
6. Ship an autonomous **Humanoid Robotics Capstone**.

---

## 📂 Repository Structure

```
project-root/
│
├── book-source/
│   └── docs/
│       ├── 01-Physical-AI-Foundations/
│       ├── 02-ROS2-Fundamentals/
│       ├── 03-Simulation-Systems/
│       ├── 04-NVIDIA-Isaac-AI/
│       ├── 05-Vision-Language-Action/
│       └── 06-Capstone/
│
├── rag-backend/
│   ├── app/ (FastAPI)
│   ├── embeddings/
│   ├── vector-store/ (Qdrant)
│   └── db/ (Neon connection)
│
├── auth/
│   └── betterauth/ (Signup/Signin)
│
├── monorepo/
│   ├── agents/ (Claude Code agents)
│   ├── skills/ (Claude reusable intelligence)
│   └── workflows/
│
├── .claude/ (Claude Code configs)
└── project-index.md
```

---

## 🚀 Technologies Used

### **Frontend / Docs**
- Docusaurus 3  
- TypeScript  
- TailwindCSS  
- AI-personalized content  
- Urdu translation system  

### **Backend**
- FastAPI  
- BetterAuth  
- Qdrant Cloud  
- Neon Serverless Postgres  

### **AI Agents + Tools**
- Claude Code  
- Spec-Kit Plus  
- Agent Skills  
- Agent Subagents  
- MCP  

### **Robotics**
- ROS 2 Humble  
- Gazebo Simulator  
- Unity Robotics  
- NVIDIA Isaac Sim  
- Isaac ROS (VSLAM / Perception / Nav2)  

---

## 📚 Chapter Structure (18 Chapters)
See **project-index.md** for full details.

---

## 🧠 How to Use This Repository with Claude Code

1. Open the repository in **Claude Code**.  
2. Claude will automatically load:
   - `project-index.md`
   - `.claude/claude.md` (after we create it)
3. Ask Claude to:
   - Generate chapters  
   - Build lessons  
   - Create RAG backend files  
   - Create ROS2 packages  
   - Generate simulations  
   - Add personalization logic  

Everything is **spec-driven**, not prompt-driven.

---

## 🔐 Authentication — BetterAuth

This project uses **BetterAuth** for:

- Signup  
- Signin  
- Asking user onboarding questions:
  - Software background
  - Robotics background
  - Hardware availability
- Storing user preferences for:
  - Personalization
  - Urdu translation
  - Learning pace

---

## 🔍 RAG Chatbot

The RAG chatbot uses:

- **OpenAI Agents SDK / ChatKit**
- **FastAPI backend**
- **Qdrant Cloud Vector DB**
- **Neon Serverless Postgres**

Features:
- Chapter-level retrieval  
- “Answer only from selected text” mode  
- Book-wide semantic search  
- Chat history  
- Personalization-aware answers  

---

## 💡 Capstone Project

You will build a simulated autonomous humanoid robot that:

- Takes natural language commands  
- Uses Whisper for speech recognition  
- Plans using an LLM  
- Navigates using Nav2  
- Perceives using Isaac ROS  
- Manipulates objects in simulation  
- Executes tasks autonomously  

---

## ⚙️ Running the Project

### Install dependencies:
```
pnpm install
```

### Run Docusaurus locally:
```
pnpm start
```

### Run RAG backend:
```
uvicorn app.main:app --reload
```

### Run Qdrant locally (optional):
```
docker-compose up
```

---

## 🧵 Feedback & Collaboration

This project is designed to be continuously improved using:

- Spec‑Kit Plus  
- Claude Code iterative development  
- Agent Skills & Reusable Intelligence  
- Structured plans & specs  

---

## 🏁 License
MIT

