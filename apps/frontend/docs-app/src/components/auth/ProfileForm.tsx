import React from 'react';
import GlassCard from '../ui/GlassCard';
import MultiSelect from '../ui/MultiSelect';
import { VALID_SOFTWARE_SKILLS, VALID_HARDWARE_ACCESS } from '../../models/user-profile';

interface ProfileFormProps {
  softwareBackground: string[];
  hardwareAccess: string[];
  comfortLevel: number;
  onSoftwareBackgroundChange: (values: string[]) => void;
  onHardwareAccessChange: (values: string[]) => void;
  onComfortLevelChange: (value: number) => void;
  title?: string;
  subtitle?: string;
}

const ProfileForm: React.FC<ProfileFormProps> = ({
  softwareBackground,
  hardwareAccess,
  comfortLevel,
  onSoftwareBackgroundChange,
  onHardwareAccessChange,
  onComfortLevelChange,
  title = 'Profile Information',
  subtitle = 'Help us personalize your learning experience'
}) => {
  return (
    <GlassCard className="w-full max-w-md">
      <div className="text-center mb-6">
        <h2 className="text-2xl font-bold text-gray-900 dark:text-white">{title}</h2>
        <p className="text-gray-600 dark:text-gray-400 mt-2">
          {subtitle}
        </p>
      </div>

      <div className="space-y-6">
        <div>
          <MultiSelect
            label="Software Background (Select all that apply)"
            options={VALID_SOFTWARE_SKILLS}
            selected={softwareBackground}
            onChange={onSoftwareBackgroundChange}
            placeholder="Select software skills..."
          />
        </div>

        <div>
          <MultiSelect
            label="Hardware Access (Select all that apply)"
            options={VALID_HARDWARE_ACCESS}
            selected={hardwareAccess}
            onChange={onHardwareAccessChange}
            placeholder="Select hardware access..."
          />
        </div>

        <div>
          <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-1">
            Comfort Level (1 = Beginner, 5 = Expert)
          </label>
          <div className="flex items-center space-x-2">
            {[1, 2, 3, 4, 5].map((level) => (
              <button
                key={level}
                type="button"
                className={`w-10 h-10 rounded-full border ${
                  comfortLevel === level
                    ? 'bg-blue-600 text-white border-blue-600'
                    : 'border-gray-300 dark:border-gray-600 text-gray-700 dark:text-gray-300 hover:bg-gray-100 dark:hover:bg-gray-700'
                }`}
                onClick={() => onComfortLevelChange(level)}
              >
                {level}
              </button>
            ))}
          </div>
          <div className="flex justify-between text-xs text-gray-500 dark:text-gray-400 mt-1">
            <span>Beginner</span>
            <span>Expert</span>
          </div>
        </div>
      </div>
    </GlassCard>
  );
};

export default ProfileForm;