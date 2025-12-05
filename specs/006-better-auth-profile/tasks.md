# Tasks: Better-Auth Sign-Up/Sign-In with Software & Hardware Background Capture

**Feature**: Better-Auth Profile System
**Branch**: 006-better-auth-profile
**Created**: 2025-12-06
**Status**: Generated from spec, plan, and design docs

## Implementation Strategy

This feature implements a user authentication system with Better-Auth that captures user background information (software skills, hardware access, and comfort level) during sign-up. The implementation follows an MVP-first approach with incremental delivery:

- **MVP Scope**: User Story 1 (New User Sign-Up with Profile Capture) with basic authentication and profile storage
- **Phase 2**: User Story 2 (Profile Management Post-Sign-Up)
- **Phase 3**: User Story 3 (Content Personalization Based on Profile)
- **Final Phase**: Polish and cross-cutting concerns

Each user story is designed to be independently testable and deliver value to users.

## Dependencies

- User Story 1 (P1) must be completed before User Story 2 (P2)
- User Story 2 (P2) must be completed before User Story 3 (P3)
- Foundational phase tasks must be completed before any user story phases

## Parallel Execution Examples

Per User Story 1:
- [P] T010-T015 can execute in parallel (UI components)
- [P] T020-T025 can execute in parallel (API endpoints)
- [P] T030-T035 can execute in parallel (Database setup and services)

## Phase 1: Setup

Initialize project structure and configure dependencies for Better-Auth integration with Docusaurus.

### Tasks

- [x] T001 Set up Better-Auth configuration in `frontend/better-auth/config.ts`
- [x] T002 Configure Better-Auth with Neon Postgres adapter in `frontend/better-auth/config.ts`
- [x] T003 Set up GitHub OAuth provider in Better-Auth configuration
- [x] T004 Install Better-Auth dependencies: `better-auth`, `@better-auth/node`, `@better-auth/postgres-adapter`
- [x] T005 Create `.env.example` with `BETTER_AUTH_SECRET`, `GITHUB_CLIENT_ID`, `GITHUB_CLIENT_SECRET`, `DATABASE_URL`
- [x] T006 Configure Docusaurus plugin for Better-Auth integration in `docusaurus.config.js`

## Phase 2: Foundational

Implement foundational components that are required for all user stories.

### Tasks

- [x] T007 Create database schema for user_profiles table in Neon Postgres
- [x] T008 Implement UserProfile entity model in `frontend/src/models/user-profile.ts`
- [x] T009 Create AuthContext for managing authentication state in `frontend/src/contexts/AuthContext.tsx`
- [x] T010 Create useAuth hook for authentication in `frontend/src/hooks/useAuth.ts`
- [x] T011 Create GlassCard UI component with glassmorphism design in `frontend/src/components/ui/GlassCard.tsx`
- [x] T012 Create MultiSelect UI component for skills/hardware selection in `frontend/src/components/ui/MultiSelect.tsx`
- [x] T013 Set up API route structure for profile management in `frontend/src/pages/api/profile.ts`

## Phase 3: User Story 1 - New User Sign-Up with Profile Capture (Priority: P1)

A new learner visits the Physical-AI textbook website and creates an account. During sign-up, they provide their email/password or use GitHub OAuth, then complete a profile form capturing their software background (Python, ROS 2, C++, PyTorch, LLM, None), hardware access (Jetson, RealSense, Unitree, None), and comfort level (1-5 Likert scale). The user is then redirected to a welcome page and finally to the textbook home.

**Independent Test**: Can be fully tested by creating a new account with profile information and verifying that the data is correctly stored and accessible for content personalization.

### Story Goal
Implement complete sign-up flow with profile capture functionality, including email/password and GitHub OAuth authentication, profile form collection, and proper redirects.

### Independent Test Criteria
- New users can sign up via email/password or GitHub OAuth
- Profile information is collected during sign-up
- Profile data is stored in Neon Postgres
- User is redirected through the specified flow: `/signup` → profile questions → `/welcome` → textbook home
- Default values are used when profile is skipped

### Tasks

- [x] T014 [P] [US1] Create SignUpForm component with email/password fields in `frontend/src/components/auth/SignUpForm.tsx`
- [x] T015 [P] [US1] Create GitHub OAuth sign-up button in `frontend/src/components/auth/GitHubOAuthButton.tsx`
- [x] T016 [P] [US1] Create ProfileForm component for software background collection in `frontend/src/components/auth/ProfileForm.tsx`
- [x] T017 [P] [US1] Create ProfileForm component for hardware access collection in `frontend/src/components/auth/ProfileForm.tsx`
- [x] T018 [P] [US1] Create ProfileForm component for comfort level selection in `frontend/src/components/auth/ProfileForm.tsx`
- [x] T019 [US1] Create signup page with form integration in `frontend/src/pages/signup.tsx`
- [x] T020 [P] [US1] Implement POST /api/profile endpoint for profile creation in `frontend/src/pages/api/profile.ts`
- [x] T021 [P] [US1] Implement GET /api/profile endpoint for profile retrieval in `frontend/src/pages/api/profile.ts`
- [x] T022 [US1] Create welcome page after sign-up in `frontend/src/pages/welcome.tsx`
- [x] T023 [US1] Implement client-side redirect flow: `/signup` → profile questions → `/welcome` → textbook home
- [x] T024 [US1] Create AuthLayout component for authentication pages in `frontend/src/components/auth/AuthLayout.tsx`
- [x] T025 [US1] Implement profile validation logic for software background, hardware access, and comfort level
- [x] T026 [US1] Create UserProfile service for database operations in `frontend/src/services/UserProfileService.ts`
- [x] T027 [US1] Implement optional profile completion with default values (comfort: 1, hardware: ["None"], software: ["None"])
- [x] T028 [US1] Add WCAG 2.2 AA compliance to signup and profile forms
- [x] T029 [US1] Implement error handling for profile creation/validation failures

## Phase 4: User Story 2 - Profile Management Post-Sign-Up (Priority: P2)

An existing user accesses their profile page to update their software background, hardware access, and comfort level. They can modify their information and save changes that will be reflected in content personalization.

**Independent Test**: Can be tested by updating profile information and verifying the changes are persisted and retrievable.

### Story Goal
Implement profile management functionality for existing authenticated users to update their background information.

### Independent Test Criteria
- Authenticated users can access their profile page
- Users can update their software background, hardware access, and comfort level
- Changes are persisted to the database
- Updated information is available for content personalization

### Tasks

- [ ] T030 [US2] Create profile page for existing users in `frontend/src/pages/profile.tsx`
- [ ] T031 [US2] Implement PUT /api/profile endpoint for profile updates in `frontend/src/pages/api/profile.ts`
- [ ] T032 [US2] Update UserProfile service to support profile updates in `frontend/src/services/UserProfileService.ts`
- [ ] T033 [US2] Create ProfileForm component for editing existing profiles in `frontend/src/components/auth/ProfileForm.tsx`
- [ ] T034 [US2] Implement authentication check for profile access (401 for unauthenticated)
- [ ] T035 [US2] Add validation for profile update requests
- [ ] T036 [US2] Implement UI feedback for successful profile updates
- [ ] T037 [US2] Add audit trail for profile changes (updated_at timestamp)

## Phase 5: User Story 3 - Content Personalization Based on Profile (Priority: P3)

The system uses the user's profile information to customize content delivery, showing appropriate chapter difficulty, code examples, and hardware warnings based on their background and comfort level.

**Independent Test**: Can be tested by verifying that different users with different profiles see different content based on their profile information.

### Story Goal
Implement content personalization based on user profile information to deliver appropriate learning materials.

### Independent Test Criteria
- Users with different comfort levels see content of appropriate difficulty
- Users with different software backgrounds see relevant code examples
- Users with different hardware access see appropriate hardware warnings/content

### Tasks

- [ ] T038 [US3] Create content personalization service in `frontend/src/services/ContentPersonalizationService.ts`
- [ ] T039 [US3] Implement logic to adjust content difficulty based on comfort level
- [ ] T040 [US3] Implement logic to select appropriate code examples based on software background
- [ ] T041 [US3] Implement logic to show hardware warnings based on hardware access
- [ ] T042 [US3] Create content filtering components that adapt based on user profile
- [ ] T043 [US3] Integrate personalization service with content rendering
- [ ] T044 [US3] Add A/B testing capability to measure personalization effectiveness
- [ ] T045 [US3] Implement analytics tracking for personalization usage

## Phase 6: Polish & Cross-Cutting Concerns

Implement final touches, security measures, accessibility features, and quality improvements.

### Tasks

- [ ] T046 Implement comprehensive error handling and user feedback
- [ ] T047 Add loading states and accessibility improvements to all auth components
- [ ] T048 Implement rate limiting for authentication endpoints
- [ ] T049 Add comprehensive logging for authentication and profile operations
- [ ] T050 Create documentation for the authentication and profile system
- [ ] T051 Implement Playwright tests for sign-up flow (≤ 45 seconds target)
- [ ] T052 Implement automated tests for profile data persistence and retrieval
- [ ] T053 Perform security review of authentication implementation
- [ ] T054 Optimize performance of profile database queries
- [ ] T055 Add analytics for user drop-off rates during sign-up (≥ 95% target)