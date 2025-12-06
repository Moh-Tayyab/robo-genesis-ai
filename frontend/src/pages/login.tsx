import React from 'react';
import LoginForm from '@site/src/components/LoginForm';

const LoginPage: React.FC = () => {
  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <LoginForm />
        </div>
      </div>
    </div>
  );
};

export default LoginPage;