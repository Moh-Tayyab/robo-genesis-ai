import React from 'react';

// Import custom components
import CodeExample from '@site/src/components/learning/CodeExample';
import ConceptCallout from '@site/src/components/learning/ConceptCallout';
import ExerciseBlock from '@site/src/components/learning/ExerciseBlock';
import LearningObjectives from '@site/src/components/LearningObjectives';
import Prerequisites from '@site/src/components/Prerequisites';

// Extend the default MDX components
const MDXComponents = {
  // Register custom components to be available in all MDX files
  CodeExample,
  ConceptCallout,
  ExerciseBlock,
  LearningObjectives,
  Prerequisites,
};

export default MDXComponents;