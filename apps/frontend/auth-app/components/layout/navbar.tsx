'use client';

import Link from 'next/link';
import { useSessionContext } from '@/lib/providers/session-provider';
import { signOut } from '@/lib/auth/signout';
import { Button, DropdownMenu, DropdownMenuContent, DropdownMenuItem, DropdownMenuTrigger, DropdownMenuSeparator, Avatar, AvatarFallback, AvatarImage } from '@repo/ui';
import { Loader2, LogOut, User, Settings } from 'lucide-react';

const docsUrl = process.env.NEXT_PUBLIC_DOCS_URL || 'http://localhost:3000';

export function Navbar() {
  const { user, isLoading, isAuthenticated } = useSessionContext();

  return (
    <nav className="border-b border-gray-200 dark:border-gray-800 bg-white dark:bg-gray-950">
      <div className="max-w-7xl mx-auto px-4 sm:px-6 lg:px-8">
        <div className="flex justify-between h-16 items-center">
          <div className="flex items-center">
            <Link href={docsUrl} className="text-xl font-bold text-blue-600">
              Robo Genesis Ai
            </Link>
          </div>

          <div className="flex items-center space-x-4">
            {isLoading ? (
              <Loader2 className="h-5 w-5 animate-spin text-gray-400" />
            ) : isAuthenticated && user ? (
              <DropdownMenu>
                <DropdownMenuTrigger asChild>
                  <Button
                    variant="ghost"
                    className="relative h-8 w-8 rounded-full flex items-center justify-center"
                  >
                    <Avatar className="h-8 w-8">
                      <AvatarImage
                        src={user.email ? `https://www.gravatar.com/avatar/${user.email}?d=identicon` : undefined}
                        alt={user.name || user.email}
                      />
                      <AvatarFallback>
                        <User className="h-4 w-4" />
                      </AvatarFallback>
                    </Avatar>
                  </Button>
                </DropdownMenuTrigger>
                <DropdownMenuContent className="w-56" align="end" forceMount>
                  <div className="flex flex-col space-y-2 p-2">
                    <div className="text-xs font-medium text-gray-500 dark:text-gray-400">
                      Signed in as
                    </div>
                    <div className="text-sm font-medium">
                      {user.name || user.email}
                    </div>
                    <div className="text-xs text-gray-500 dark:text-gray-400 truncate">
                      {user.email}
                    </div>
                  </div>
                  <DropdownMenuSeparator />
                  <DropdownMenuItem asChild>
                    <Link href="/account" className="flex items-center">
                      <User className="mr-2 h-4 w-4" />
                      <span>Profile</span>
                    </Link>
                  </DropdownMenuItem>
                  <DropdownMenuItem asChild>
                    <Link href="/settings" className="flex items-center">
                      <Settings className="mr-2 h-4 w-4" />
                      <span>Settings</span>
                    </Link>
                  </DropdownMenuItem>
                  <DropdownMenuSeparator />
                  <DropdownMenuItem
                    onClick={() => signOut()}
                    className="flex items-center text-red-600 focus:text-red-600"
                  >
                    <LogOut className="mr-2 h-4 w-4" />
                    <span>Sign out</span>
                  </DropdownMenuItem>
                </DropdownMenuContent>
              </DropdownMenu>
            ) : (
              <>
                <Link href="/signin">
                  <Button variant="ghost" size="sm">Sign in</Button>
                </Link>
                <Link href="/signup">
                  <Button size="sm">Get started</Button>
                </Link>
              </>
            )}
          </div>
        </div>
      </div>
    </nav>
  );
}
