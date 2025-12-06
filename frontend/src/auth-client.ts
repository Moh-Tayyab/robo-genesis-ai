import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: typeof window !== 'undefined' ? window.location.origin : 'http://localhost:3000',
  fetchUser: true,
  fetcher: (url, options) => {
    // Add base API URL for the auth endpoints
    return fetch(`/api/auth${url}`, options);
  }
});

export const { auth, useSession } = authClient;