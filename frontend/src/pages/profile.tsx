import React, { useEffect, useState } from 'react';
import { useAuth } from '@site/src/components/AuthProvider';

const ProfilePage: React.FC = () => {
  const { user, loading, fetchUserBackground } = useAuth();
  const [editing, setEditing] = useState(false);
  const [editData, setEditData] = useState({
    programming_experience: '',
    os_preference: '',
    gpu_available: false,
    preferred_language: '',
    learning_goals: '',
    hardware_background: '',
    software_background: '',
  });

  useEffect(() => {
    if (user && !user.profile_completed) {
      fetchUserBackground();
    }

    if (user) {
      setEditData({
        programming_experience: user.programming_experience || '',
        os_preference: user.os_preference || '',
        gpu_available: user.gpu_available || false,
        preferred_language: user.preferred_language || '',
        learning_goals: user.learning_goals || '',
        hardware_background: user.hardware_background || '',
        software_background: user.software_background || '',
      });
    }
  }, [user, fetchUserBackground]);

  const handleEditToggle = () => {
    if (editing) {
      // Save changes
      // In a real implementation, we would call updateUserBackground here
    }
    setEditing(!editing);
  };

  if (loading) {
    return (
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <p>Loading profile...</p>
          </div>
        </div>
      </div>
    );
  }

  if (!user) {
    return (
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <p>Please <a href="/login">log in</a> to view your profile.</p>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--8 col--offset-2">
          <h1>User Profile</h1>

          <div className="card">
            <div className="card__header">
              <h3>Account Information</h3>
            </div>
            <div className="card__body">
              <p><strong>Email:</strong> {user.email}</p>
              <p><strong>Name:</strong> {user.name || 'Not provided'}</p>
              <p><strong>Profile Completed:</strong> {user.profile_completed ? 'Yes' : 'No'}</p>
            </div>
          </div>

          <div className="card margin-vert--lg">
            <div className="card__header">
              <h3>Background Information</h3>
              <button
                className="button button--primary button--sm"
                onClick={handleEditToggle}
              >
                {editing ? 'Save Changes' : 'Edit Profile'}
              </button>
            </div>
            <div className="card__body">
              {editing ? (
                <form>
                  <div className="form-group">
                    <label htmlFor="programming_experience">Programming Experience</label>
                    <select
                      id="programming_experience"
                      name="programming_experience"
                      value={editData.programming_experience}
                      onChange={(e) => setEditData({...editData, programming_experience: e.target.value})}
                    >
                      <option value="">Select level</option>
                      <option value="beginner">Beginner</option>
                      <option value="intermediate">Intermediate</option>
                      <option value="advanced">Advanced</option>
                      <option value="expert">Expert</option>
                    </select>
                  </div>

                  <div className="form-group">
                    <label htmlFor="os_preference">OS Preference</label>
                    <select
                      id="os_preference"
                      name="os_preference"
                      value={editData.os_preference}
                      onChange={(e) => setEditData({...editData, os_preference: e.target.value})}
                    >
                      <option value="">Select OS</option>
                      <option value="windows">Windows</option>
                      <option value="macos">macOS</option>
                      <option value="linux">Linux</option>
                    </select>
                  </div>

                  <div className="form-group">
                    <label>
                      <input
                        type="checkbox"
                        checked={editData.gpu_available}
                        onChange={(e) => setEditData({...editData, gpu_available: e.target.checked})}
                      />
                      I have access to a GPU
                    </label>
                  </div>

                  <div className="form-group">
                    <label htmlFor="preferred_language">Preferred Language</label>
                    <select
                      id="preferred_language"
                      name="preferred_language"
                      value={editData.preferred_language}
                      onChange={(e) => setEditData({...editData, preferred_language: e.target.value})}
                    >
                      <option value="">Select language</option>
                      <option value="python">Python</option>
                      <option value="typescript">TypeScript</option>
                      <option value="javascript">JavaScript</option>
                      <option value="cpp">C++</option>
                      <option value="rust">Rust</option>
                    </select>
                  </div>

                  <div className="form-group">
                    <label htmlFor="hardware_background">Hardware Background</label>
                    <select
                      id="hardware_background"
                      name="hardware_background"
                      value={editData.hardware_background}
                      onChange={(e) => setEditData({...editData, hardware_background: e.target.value})}
                    >
                      <option value="">Select background</option>
                      <option value="none">None</option>
                      <option value="maker">Maker</option>
                      <option value="engineer">Engineer</option>
                      <option value="researcher">Researcher</option>
                    </select>
                  </div>

                  <div className="form-group">
                    <label htmlFor="software_background">Software Background</label>
                    <select
                      id="software_background"
                      name="software_background"
                      value={editData.software_background}
                      onChange={(e) => setEditData({...editData, software_background: e.target.value})}
                    >
                      <option value="">Select background</option>
                      <option value="none">None</option>
                      <option value="developer">Developer</option>
                      <option value="engineer">Engineer</option>
                      <option value="researcher">Researcher</option>
                    </select>
                  </div>

                  <div className="form-group">
                    <label htmlFor="learning_goals">Learning Goals</label>
                    <textarea
                      id="learning_goals"
                      name="learning_goals"
                      value={editData.learning_goals}
                      onChange={(e) => setEditData({...editData, learning_goals: e.target.value})}
                      rows={4}
                    />
                  </div>
                </form>
              ) : (
                <div>
                  <p><strong>Programming Experience:</strong> {user.programming_experience || 'Not specified'}</p>
                  <p><strong>OS Preference:</strong> {user.os_preference || 'Not specified'}</p>
                  <p><strong>GPU Available:</strong> {user.gpu_available ? 'Yes' : 'No'}</p>
                  <p><strong>Preferred Language:</strong> {user.preferred_language || 'Not specified'}</p>
                  <p><strong>Hardware Background:</strong> {user.hardware_background || 'Not specified'}</p>
                  <p><strong>Software Background:</strong> {user.software_background || 'Not specified'}</p>
                  <p><strong>Learning Goals:</strong> {user.learning_goals || 'Not specified'}</p>
                </div>
              )}
            </div>
          </div>
        </div>
      </div>
    </div>
  );
};

export default ProfilePage;