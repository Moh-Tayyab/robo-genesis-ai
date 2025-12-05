// UserProfile entity model
export interface UserProfile {
  userId: string; // UUID from Better-Auth user.id
  softwareBackground: string[]; // ["Python", "ROS 2", "C++", "PyTorch", "LLM", "None"]
  hardwareAccess: string[]; // ["Jetson", "RealSense", "Unitree", "None"]
  comfortLevel: number; // 1-5 Likert scale (1=beginner, 5=expert)
  createdAt: Date;
  updatedAt: Date;
  isProfileComplete: boolean;
}

// UserProfile creation/update DTO
export interface UserProfileInput {
  softwareBackground?: string[]; // ["Python", "ROS 2", "C++", "PyTorch", "LLM", "None"]
  hardwareAccess?: string[]; // ["Jetson", "RealSense", "Unitree", "None"]
  comfortLevel?: number; // 1-5 Likert scale (1=beginner, 5=expert)
}

// Valid values for validation
export const VALID_SOFTWARE_SKILLS = ["Python", "ROS 2", "C++", "PyTorch", "LLM", "None"] as const;
export const VALID_HARDWARE_ACCESS = ["Jetson", "RealSense", "Unitree", "None"] as const;
export const COMFORT_LEVEL_MIN = 1;
export const COMFORT_LEVEL_MAX = 5;

// Default values when profile is skipped
export const DEFAULT_PROFILE: Omit<UserProfile, 'userId' | 'createdAt' | 'updatedAt'> = {
  softwareBackground: ["None"],
  hardwareAccess: ["None"],
  comfortLevel: 1,
  isProfileComplete: false
};