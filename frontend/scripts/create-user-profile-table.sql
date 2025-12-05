-- Create user_profiles table for storing user background information
CREATE TABLE IF NOT EXISTS user_profiles (
    user_id UUID PRIMARY KEY REFERENCES auth_accounts(id) ON DELETE CASCADE,
    software_background TEXT[] DEFAULT '{}',
    hardware_access TEXT[] DEFAULT '{}',
    comfort_level INTEGER CHECK (comfort_level >= 1 AND comfort_level <= 5),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    is_profile_complete BOOLEAN DEFAULT FALSE,
    CONSTRAINT fk_user_id FOREIGN KEY (user_id) REFERENCES auth_accounts(id)
);

-- Create indexes for performance
CREATE INDEX IF NOT EXISTS idx_user_profiles_comfort_level ON user_profiles(comfort_level);
CREATE INDEX IF NOT EXISTS idx_user_profiles_created_at ON user_profiles(created_at);

-- Add updated_at trigger to automatically update timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_user_profiles_updated_at
    BEFORE UPDATE ON user_profiles
    FOR EACH ROW
    EXECUTE FUNCTION update_updated_at_column();