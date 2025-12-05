import React, { useState } from 'react';
import clsx from 'clsx';

interface MultiSelectProps {
  options: string[];
  selected: string[];
  onChange: (selected: string[]) => void;
  placeholder?: string;
  label?: string;
  className?: string;
  disabled?: boolean;
}

const MultiSelect: React.FC<MultiSelectProps> = ({
  options,
  selected,
  onChange,
  placeholder = 'Select options...',
  label,
  className,
  disabled = false,
}) => {
  const [isOpen, setIsOpen] = useState(false);

  const toggleOption = (option: string) => {
    if (selected.includes(option)) {
      onChange(selected.filter(item => item !== option));
    } else {
      onChange([...selected, option]);
    }
  };

  const selectAll = () => {
    onChange(options);
  };

  const clearAll = () => {
    onChange([]);
  };

  return (
    <div className={clsx('relative', className)}>
      {label && (
        <label className="block text-sm font-medium text-gray-700 dark:text-gray-300 mb-2">
          {label}
        </label>
      )}
      <div
        className={clsx(
          'min-h-12 w-full rounded-md border border-gray-300 bg-white dark:bg-gray-800 dark:border-gray-600',
          'px-3 py-2 text-left cursor-pointer focus:outline-none focus:ring-2 focus:ring-blue-500 focus:border-blue-500',
          'flex flex-wrap gap-2 items-center',
          disabled && 'bg-gray-100 dark:bg-gray-700 cursor-not-allowed'
        )}
        onClick={() => !disabled && setIsOpen(!isOpen)}
      >
        {selected.length === 0 ? (
          <span className="text-gray-500 dark:text-gray-400 text-sm">{placeholder}</span>
        ) : (
          <div className="flex flex-wrap gap-1">
            {selected.map((item) => (
              <span
                key={item}
                className="inline-flex items-center gap-1 px-2 py-1 rounded-full text-xs font-medium bg-blue-100 text-blue-800 dark:bg-blue-900 dark:text-blue-100"
              >
                {item}
                <button
                  type="button"
                  className="ml-1 rounded-full hover:bg-blue-200 dark:hover:bg-blue-800"
                  onClick={(e) => {
                    e.stopPropagation();
                    toggleOption(item);
                  }}
                  disabled={disabled}
                >
                  ×
                </button>
              </span>
            ))}
          </div>
        )}
        <button
          type="button"
          className="ml-auto text-gray-400 hover:text-gray-600 dark:text-gray-500 dark:hover:text-gray-300"
          onClick={(e) => {
            e.stopPropagation();
            setIsOpen(!isOpen);
          }}
          disabled={disabled}
        >
          <svg
            className={`w-5 h-5 transform transition-transform ${isOpen ? 'rotate-180' : ''}`}
            xmlns="http://www.w3.org/2000/svg"
            viewBox="0 0 20 20"
            fill="currentColor"
          >
            <path fillRule="evenodd" d="M5.293 7.293a1 1 0 011.414 0L10 10.586l3.293-3.293a1 1 0 111.414 1.414l-4 4a1 1 0 01-1.414 0l-4-4a1 1 0 010-1.414z" clipRule="evenodd" />
          </svg>
        </button>
      </div>

      {isOpen && !disabled && (
        <div className="absolute z-10 w-full mt-1 bg-white dark:bg-gray-800 border border-gray-300 dark:border-gray-600 rounded-md shadow-lg max-h-60 overflow-auto">
          <div className="p-2 border-b border-gray-200 dark:border-gray-700 flex justify-between">
            <button
              type="button"
              className="text-xs px-2 py-1 rounded bg-gray-100 dark:bg-gray-700 hover:bg-gray-200 dark:hover:bg-gray-600"
              onClick={(e) => {
                e.stopPropagation();
                selectAll();
              }}
            >
              Select All
            </button>
            <button
              type="button"
              className="text-xs px-2 py-1 rounded bg-gray-100 dark:bg-gray-700 hover:bg-gray-200 dark:hover:bg-gray-600"
              onClick={(e) => {
                e.stopPropagation();
                clearAll();
              }}
            >
              Clear All
            </button>
          </div>
          <div className="py-1">
            {options.map((option) => (
              <div
                key={option}
                className={clsx(
                  'px-4 py-2 text-sm cursor-pointer flex items-center',
                  'hover:bg-gray-100 dark:hover:bg-gray-700',
                  selected.includes(option) && 'bg-blue-50 dark:bg-blue-900/50'
                )}
                onClick={() => toggleOption(option)}
              >
                <input
                  type="checkbox"
                  checked={selected.includes(option)}
                  readOnly
                  className="mr-2 rounded text-blue-600 focus:ring-blue-500"
                />
                {option}
              </div>
            ))}
          </div>
        </div>
      )}
    </div>
  );
};

export default MultiSelect;