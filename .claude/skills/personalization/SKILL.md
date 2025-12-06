# Personalization Skill

## Description
Adapts educational content based on user background, experience level, and preferences.

## Parameters
- `content`: The content to personalize
- `experience_level`: User's experience (beginner, intermediate, advanced, expert)
- `programming_language`: Preferred language (python, typescript, cpp, etc.)
- `background`: Professional background (hardware, software, research, etc.)
- `learning_goals`: User's learning objectives

## Usage Examples
```
skill:personalization
experience_level: beginner
programming_language: python
background: hardware
learning_goals: practical implementation
content: |
  This section explains inverse kinematics using mathematical formulas...
```

## Output
- Content adapted to user's experience level
- Examples in preferred programming language
- Context relevant to user's background
- Focus aligned with learning goals