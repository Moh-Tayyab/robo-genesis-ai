import React from 'react';
import SignupForm from '@site/src/components/SignupForm';

const ProfileSetupPage: React.FC = () => {
  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <SignupForm />
        </div>
      </div>
    </div>
  );
};

export default ProfileSetupPage;