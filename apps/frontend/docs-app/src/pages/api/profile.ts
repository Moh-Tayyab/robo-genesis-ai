import { NextApiRequest, NextApiResponse } from 'next';
import { UserProfile, DEFAULT_PROFILE } from '../../models/user-profile';

// This file represents the API route structure for profile management
// In a real implementation, this would be part of a Next.js API route
// For Docusaurus, we'd need a separate backend service to handle these

// Mock data store for demonstration purposes
let mockProfiles: Record<string, UserProfile> = {};

interface ProfileRequestBody {
  softwareBackground?: string[];
  hardwareAccess?: string[];
  comfortLevel?: number;
}

// Handler for profile API routes
export default function profileHandler(req: NextApiRequest, res: NextApiResponse) {
  const { method } = req;

  switch (method) {
    case 'GET':
      return handleGET(req, res);
    case 'POST':
      return handlePOST(req, res);
    case 'PUT':
      return handlePUT(req, res);
    default:
      res.setHeader('Allow', ['GET', 'POST', 'PUT']);
      res.status(405).end(`Method ${method} Not Allowed`);
  }
}

// GET /api/profile - Retrieve user profile
const handleGET = (req: NextApiRequest, res: NextApiResponse) => {
  try {
    // In a real implementation, we would get the user ID from the auth token
    const userId = req.headers.authorization; // Simplified for example

    if (!userId) {
      return res.status(401).json({
        error: {
          code: 'UNAUTHENTICATED',
          message: 'User is not authenticated'
        }
      });
    }

    // Look up the user's profile
    const profile = mockProfiles[userId as string];

    if (!profile) {
      // Return default profile if none exists
      return res.status(200).json({
        userId: userId,
        ...DEFAULT_PROFILE,
        createdAt: new Date(),
        updatedAt: new Date(),
        isProfileComplete: false
      });
    }

    return res.status(200).json(profile);
  } catch (error) {
    console.error('Error retrieving profile:', error);
    return res.status(500).json({
      error: {
        code: 'INTERNAL_ERROR',
        message: 'Internal server error occurred'
      }
    });
  }
};

// POST /api/profile - Create or update user profile
const handlePOST = (req: NextApiRequest, res: NextApiResponse) => {
  try {
    // In a real implementation, we would get the user ID from the auth token
    const userId = req.headers.authorization; // Simplified for example

    if (!userId) {
      return res.status(401).json({
        error: {
          code: 'UNAUTHENTICATED',
          message: 'User is not authenticated'
        }
      });
    }

    const { softwareBackground, hardwareAccess, comfortLevel }: ProfileRequestBody = req.body;

    // Validate input
    if (softwareBackground && !Array.isArray(softwareBackground)) {
      return res.status(400).json({
        error: {
          code: 'VALIDATION_ERROR',
          message: 'softwareBackground must be an array'
        }
      });
    }

    if (hardwareAccess && !Array.isArray(hardwareAccess)) {
      return res.status(400).json({
        error: {
          code: 'VALIDATION_ERROR',
          message: 'hardwareAccess must be an array'
        }
      });
    }

    if (comfortLevel && (typeof comfortLevel !== 'number' || comfortLevel < 1 || comfortLevel > 5)) {
      return res.status(400).json({
        error: {
          code: 'VALIDATION_ERROR',
          message: 'comfortLevel must be a number between 1 and 5'
        }
      });
    }

    // Create or update profile
    const profile: UserProfile = {
      userId: userId as string,
      softwareBackground: softwareBackground || DEFAULT_PROFILE.softwareBackground,
      hardwareAccess: hardwareAccess || DEFAULT_PROFILE.hardwareAccess,
      comfortLevel: comfortLevel || DEFAULT_PROFILE.comfortLevel,
      createdAt: mockProfiles[userId as string]?.createdAt || new Date(),
      updatedAt: new Date(),
      isProfileComplete: true
    };

    mockProfiles[userId as string] = profile;

    return res.status(200).json(profile);
  } catch (error) {
    console.error('Error creating profile:', error);
    return res.status(500).json({
      error: {
        code: 'INTERNAL_ERROR',
        message: 'Internal server error occurred'
      }
    });
  }
};

// PUT /api/profile - Update user profile
const handlePUT = (req: NextApiRequest, res: NextApiResponse) => {
  try {
    // In a real implementation, we would get the user ID from the auth token
    const userId = req.headers.authorization; // Simplified for example

    if (!userId) {
      return res.status(401).json({
        error: {
          code: 'UNAUTHENTICATED',
          message: 'User is not authenticated'
        }
      });
    }

    const { softwareBackground, hardwareAccess, comfortLevel }: ProfileRequestBody = req.body;

    // Validate input
    if (softwareBackground && !Array.isArray(softwareBackground)) {
      return res.status(400).json({
        error: {
          code: 'VALIDATION_ERROR',
          message: 'softwareBackground must be an array'
        }
      });
    }

    if (hardwareAccess && !Array.isArray(hardwareAccess)) {
      return res.status(400).json({
        error: {
          code: 'VALIDATION_ERROR',
          message: 'hardwareAccess must be an array'
        }
      });
    }

    if (comfortLevel && (typeof comfortLevel !== 'number' || comfortLevel < 1 || comfortLevel > 5)) {
      return res.status(400).json({
        error: {
          code: 'VALIDATION_ERROR',
          message: 'comfortLevel must be a number between 1 and 5'
        }
      });
    }

    // Update existing profile
    const existingProfile = mockProfiles[userId as string];
    if (!existingProfile) {
      return res.status(404).json({
        error: {
          code: 'NOT_FOUND',
          message: 'Profile not found'
        }
      });
    }

    const updatedProfile: UserProfile = {
      ...existingProfile,
      softwareBackground: softwareBackground || existingProfile.softwareBackground,
      hardwareAccess: hardwareAccess || existingProfile.hardwareAccess,
      comfortLevel: comfortLevel || existingProfile.comfortLevel,
      updatedAt: new Date(),
      isProfileComplete: true
    };

    mockProfiles[userId as string] = updatedProfile;

    return res.status(200).json(updatedProfile);
  } catch (error) {
    console.error('Error updating profile:', error);
    return res.status(500).json({
      error: {
        code: 'INTERNAL_ERROR',
        message: 'Internal server error occurred'
      }
    });
  }
};