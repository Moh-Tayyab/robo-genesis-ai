import type { Metadata } from "next";
import { Inter } from 'next/font/google';
import "./globals.css";
import { SessionProviderWrapper } from './session-provider-wrapper';

const inter = Inter({ subsets: ['latin'] });

export const metadata: Metadata = {
  title: "Robo Genesis Ai Auth",
  description: "Authentication for Robo Genesis Ai Physical AI & Humanoid Robotics Textbook",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en">
      <body className={inter.className}>
        <SessionProviderWrapper>
          {children}
          {/* Floating Chatbot Icon */}
          <div className="fixed bottom-6 right-6 z-50">
            <button className="w-14 h-14 bg-blue-600 text-white rounded-full shadow-lg flex items-center justify-center hover:bg-blue-700 transition-all transform hover:scale-105">
              <svg xmlns="http://www.w3.org/2000/svg" width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round" className="lucide lucide-message-circle">
                <path d="M7.9 20A9 9 0 1 0 4 16.1L2 22Z"/>
              </svg>
            </button>
          </div>
        </SessionProviderWrapper>
      </body>
    </html>
  );
}
