import React, { HTMLAttributes } from 'react';
import clsx from 'clsx';

interface GlassCardProps extends HTMLAttributes<HTMLDivElement> {
  children: React.ReactNode;
  className?: string;
  elevated?: boolean;
}

const GlassCard: React.FC<GlassCardProps> = ({
  children,
  className,
  elevated = false,
  ...props
}) => {
  return (
    <div
      className={clsx(
        'relative rounded-xl border border-gray-200/30 bg-white/10 bg-clip-padding backdrop-blur-xl',
        'dark:border-gray-700/50 dark:bg-gray-900/30',
        elevated ? 'shadow-2xl' : 'shadow-lg',
        'p-6',
        className
      )}
      {...props}
    >
      {children}
    </div>
  );
};

export default GlassCard;