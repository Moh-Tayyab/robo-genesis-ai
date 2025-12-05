# Feature Specification: Better-Auth Sign-Up / Sign-In with Software & Hardware Background Capture

**Feature Branch**: `006-better-auth-profile`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Bonus Feature: Better-Auth Sign-Up / Sign-In with Software & Hardware Background Capture (50 hackathon bonus points)

Audience: Learners accessing the Physical-AI textbook & RAG chatbot
Goal: Collect self-reported software + hardware background at sign-up to power downstream content personalisation (chapter difficulty, code examples, hardware warnings).

Business Rules
1. Only e-mail + password or GitHub OAuth (no social other than GitHub).
2. During sign-up collect:
   a. Software background (multi-select) – Python, ROS 2, C++, PyTorch, LLM, None
   b. Hardware access (multi-select) – Jetson, RealSense, Unitree, None
   c. Comfort level – 1-5 Likert (1 = beginner … 5 = expert)
3. Store answers in Neon Postgres `user_profile` table linked to Better-Auth `user.id`.
4. Profile optional post-sign-up; skip allowed (default to beginner / no hardware).
5. No PII beyond e-mail; anonymised UUID for analytics.
6. Provide `.env.example` with `BETTER_AUTH_SECRET`, `GITHUB_CLIENT_ID`, `GITHUB_CLIENT_SECRET`.
7. Expose `GET /api/profile` (own profile only) and `PUT /api/profile` (update); returns 401 if not authenticated.
8. Client-side redirect flow: `/signup` → questions → `/welcome` → textbook home.
9. UI kit: reuse existing design tokens (raisin-black, neon-cyan, neon-magenta); glassmorphism card, 24 px thin icons.
10. Accessibility: WCAG 2.2 AA, keyboard navigable, focus-visible ring.

Success Criteria
- Sign-up → profile questions → dashboard completes in ≤ 45 seconds (playwright timer).
- ≥ 95 % of new users reach profile screen (no drop-off due to UX bug).
- Data correctly persisted and retrievable via `/api/profile` (automated test).
- No secrets committed; CI passes lint + build.

Constraints
- Stack: Better-Auth v1.x, Neon Postgres, Next-JS/React (or Docusaurus plugin), Tailwind-like styled-jsx.
- Delivery: 1 week (parallel to RAG backend).
- Word limit: 3 000–5 000 for accompanying markdown documentation.

Not Building
- Multi-language onboarding (Urdu bonus separate)
- Paid SSO providers (Google, Microsoft)
- Admin dashboard for user management"

## Clarifications

### Session 2025-12-06

- Q: Should profile completion be required during sign-up? → A: Optional with skip allowed
- Q: What should be the default comfort level when profile is skipped? → A: 1 (beginner)
- Q: What should be the default hardware access when profile is skipped? → A: ["None"]
- Q: What should be the default software background when profile is skipped? → A: ["None"]
- Q: How should the API respond when a user has not completed their profile? → A: Return default values

## User Scenarios & Testing *(mandatory)*

### User Story 1 - New User Sign-Up with Profile Capture (Priority: P1)

A new learner visits the Physical-AI textbook website and creates an account. During sign-up, they provide their email/password or use GitHub OAuth, then complete a profile form capturing their software background (Python, ROS 2, C++, PyTorch, LLM, None), hardware access (Jetson, RealSense, Unitree, None), and comfort level (1-5 Likert scale). The user is then redirected to a welcome page and finally to the textbook home.

**Why this priority**: This is the core user journey that enables the personalization goal and represents the primary value proposition of the feature.

**Independent Test**: Can be fully tested by creating a new account with profile information and verifying that the data is correctly stored and accessible for content personalization.

**Acceptance Scenarios**:

1. **Given** a new user on the sign-up page, **When** they provide valid credentials and profile information, **Then** their account is created, profile is saved, and they are redirected through the specified flow.

2. **Given** a new user on the sign-up page, **When** they provide valid credentials but skip profile information, **Then** their account is created with default values (beginner, no hardware) and they are redirected through the flow.

3. **Given** a new user on the sign-up page, **When** they use GitHub OAuth, **Then** they are authenticated and presented with profile questions.

---
### User Story 2 - Profile Management Post-Sign-Up (Priority: P2)

An existing user accesses their profile page to update their software background, hardware access, and comfort level. They can modify their information and save changes that will be reflected in content personalization.

**Why this priority**: Allows users to update their information over time, maintaining accuracy of personalization data.

**Independent Test**: Can be tested by updating profile information and verifying the changes are persisted and retrievable.

**Acceptance Scenarios**:

1. **Given** an authenticated user on their profile page, **When** they update their profile information and save, **Then** the changes are persisted and available for personalization.

---
### User Story 3 - Content Personalization Based on Profile (Priority: P3)

The system uses the user's profile information to customize content delivery, showing appropriate chapter difficulty, code examples, and hardware warnings based on their background and comfort level.

**Why this priority**: This is the ultimate value proposition of the feature - personalized learning experience.

**Independent Test**: Can be tested by verifying that different users with different profiles see different content based on their profile information.

**Acceptance Scenarios**:

1. **Given** a user with beginner comfort level and Python background, **When** they access content, **Then** they see beginner-appropriate content with Python examples.

2. **Given** a user with expert comfort level and ROS 2 background, **When** they access content, **Then** they see advanced content with ROS 2 examples.

---
### Edge Cases

- What happens when a user tries to access another user's profile data?
- How does the system handle invalid profile data submissions?
- What occurs when the Neon Postgres database is temporarily unavailable during profile update?
- How does the system handle users who have not completed their profile but attempt to access personalized content?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST support email/password authentication and GitHub OAuth for user sign-up and sign-in
- **FR-002**: System MUST collect software background (Python, ROS 2, C++, PyTorch, LLM, None) during sign-up as multi-select
- **FR-003**: System MUST collect hardware access (Jetson, RealSense, Unitree, None) during sign-up as multi-select
- **FR-004**: System MUST collect comfort level (1-5 Likert scale) during sign-up
- **FR-005**: System MUST store profile data in Neon Postgres `user_profile` table linked to Better-Auth `user.id`
- **FR-006**: System MUST allow optional profile completion with default values (beginner comfort level, no hardware access)
- **FR-007**: System MUST provide GET `/api/profile` endpoint that returns authenticated user's profile data
- **FR-008**: System MUST provide PUT `/api/profile` endpoint that allows authenticated users to update their profile data
- **FR-009**: System MUST return 401 for unauthenticated access to profile endpoints
- **FR-010**: System MUST implement client-side redirect flow: `/signup` → profile questions → `/welcome` → textbook home
- **FR-011**: System MUST use existing design tokens (raisin-black, neon-cyan, neon-magenta) and glassmorphism UI components
- **FR-012**: System MUST meet WCAG 2.2 AA accessibility standards with keyboard navigation and focus-visible rings

### Key Entities *(include if feature involves data)*

- **User**: Represents a learner account, managed by Better-Auth with email, password/oauth, and unique identifier
- **UserProfile**: Contains user's software background (array of strings), hardware access (array of strings), comfort level (integer 1-5), and links to Better-Auth user.id

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Sign-up to welcome page flow completes in ≤ 45 seconds (playwright timer)
- **SC-002**: ≥ 95% of new users reach profile screen (no drop-off due to UX bug)
- **SC-003**: Profile data correctly persisted and retrievable via `/api/profile` (automated test)
- **SC-004**: No secrets committed to repository; CI passes lint + build