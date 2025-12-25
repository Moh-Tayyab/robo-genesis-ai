import { UserProfile, UserProfileInput } from '../models/user-profile';

// UserProfileService handles all user profile operations
// In a real implementation, this would communicate with the backend API
class UserProfileService {
  private static instance: UserProfileService;

  // Singleton pattern to ensure single instance
  static getInstance(): UserProfileService {
    if (!UserProfileService.instance) {
      UserProfileService.instance = new UserProfileService();
    }
    return UserProfileService.instance;
  }

  // Create or update a user profile
  async createOrUpdateProfile(userId: string, profileData: UserProfileInput): Promise<UserProfile> {
    try {
      // Validate input data before creating/updating
      const validation = this.validateProfileData(profileData);
      if (!validation.isValid) {
        throw new Error(`Invalid profile data: ${validation.errors.join(', ')}`);
      }

      // In a real implementation, this would make an API call to create/update the profile
      // For now, we'll return a mock profile
      const profile: UserProfile = {
        userId,
        softwareBackground: profileData.softwareBackground || [],
        hardwareAccess: profileData.hardwareAccess || [],
        comfortLevel: profileData.comfortLevel || 1,
        createdAt: new Date(),
        updatedAt: new Date(),
        isProfileComplete: true
      };

      return profile;
    } catch (error) {
      console.error('Error creating/updating profile:', error);
      if (error instanceof Error) {
        throw new Error(`Failed to create or update user profile: ${error.message}`);
      } else {
        throw new Error('Failed to create or update user profile: Unknown error');
      }
    }
  }

  // Get a user profile by user ID
  async getProfile(userId: string): Promise<UserProfile | null> {
    try {
      // In a real implementation, this would make an API call to retrieve the profile
      // For now, we'll return a mock profile or null if not found
      // This is a mock implementation - in reality, you'd fetch from an API
      return null;
    } catch (error) {
      console.error('Error retrieving profile:', error);
      if (error instanceof Error) {
        throw new Error(`Failed to retrieve user profile: ${error.message}`);
      } else {
        throw new Error('Failed to retrieve user profile: Unknown error');
      }
    }
  }

  // Update a user profile
  async updateProfile(userId: string, profileData: UserProfileInput): Promise<UserProfile> {
    try {
      // Validate input data before updating
      const validation = this.validateProfileData(profileData);
      if (!validation.isValid) {
        throw new Error(`Invalid profile data: ${validation.errors.join(', ')}`);
      }

      // In a real implementation, this would make an API call to update the profile
      // For now, we'll return a mock updated profile
      const profile: UserProfile = {
        userId,
        softwareBackground: profileData.softwareBackground || [],
        hardwareAccess: profileData.hardwareAccess || [],
        comfortLevel: profileData.comfortLevel || 1,
        createdAt: new Date(), // In a real implementation, this would be the original creation date
        updatedAt: new Date(),
        isProfileComplete: true
      };

      return profile;
    } catch (error) {
      console.error('Error updating profile:', error);
      if (error instanceof Error) {
        throw new Error(`Failed to update user profile: ${error.message}`);
      } else {
        throw new Error('Failed to update user profile: Unknown error');
      }
    }
  }

  // Validate profile data
  validateProfileData(profileData: UserProfileInput): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];

    // Validate software background if provided
    if (profileData.softwareBackground) {
      for (const skill of profileData.softwareBackground) {
        if (!['Python', 'ROS 2', 'C++', 'PyTorch', 'LLM', 'None'].includes(skill)) {
          errors.push(`Invalid software skill: ${skill}`);
        }
      }
    }

    // Validate hardware access if provided
    if (profileData.hardwareAccess) {
      for (const hardware of profileData.hardwareAccess) {
        if (!['Jetson', 'RealSense', 'Unitree', 'None'].includes(hardware)) {
          errors.push(`Invalid hardware access: ${hardware}`);
        }
      }
    }

    // Validate comfort level if provided
    if (profileData.comfortLevel !== undefined) {
      if (profileData.comfortLevel < 1 || profileData.comfortLevel > 5) {
        errors.push('Comfort level must be between 1 and 5');
      }
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }
}

// Export a singleton instance
export default UserProfileService.getInstance();