# Data Model: Physical_Humanoid_AI_Robotics_Course

## Overview

This document defines the data structures and relationships for the Physical_Humanoid_AI_Robotics_Course platform. It represents the entities identified in the feature specification with their attributes, relationships, and validation rules.

## Core Entities

### Course Module
**Definition**: Represents a self-contained learning unit with objectives, content, exercises, and assessments

**Attributes**:
- id: string (unique identifier)
- title: string (module title)
- description: string (brief description)
- week: integer (week number in curriculum, 1-13)
- learningObjectives: array of strings (learning objectives for the module)
- content: string (main content in Markdown format)
- prerequisites: array of Module IDs (prerequisites for this module)
- duration: integer (estimated completion time in minutes)
- difficulty: enum (beginner, intermediate, advanced)
- language: string (language code, e.g., 'en')
- createdAt: datetime
- updatedAt: datetime
- assessmentIds: array of strings (associated assessments)

**Relationships**:
- Contains many Interactive Resources
- Contains many Assessments
- Connected to Learning Path (many-to-many)

**Validation Rules**:
- Title is required and must be 5-100 characters
- Week must be between 1-13
- Duration must be positive
- Difficulty must be one of the enum values
- Language must be supported (currently 'en')

### User Profile
**Definition**: Represents different types of learners (student, researcher, engineer) with varying skill levels

**Attributes**:
- id: string (unique identifier)
- name: string (user's name)
- email: string (user's email, unique)
- role: enum (student, researcher, engineer)
- skillLevel: enum (beginner, intermediate, advanced)
- preferredLanguage: string (language code)
- createdAt: datetime
- lastActive: datetime
- completedModules: array of Module IDs
- progress: float (0-1, percentage of course completed)

**Relationships**:
- Enrolls in many Learning Paths
- Submits many Assessments
- Tracks progress in many Course Modules

**Validation Rules**:
- Email is required and must be valid email format
- Email must be unique
- Role must be one of the enum values
- Skill level must be one of the enum values
- Progress must be between 0 and 1

### Learning Path
**Definition**: Represents a sequence of modules tailored to specific user needs and skill levels

**Attributes**:
- id: string (unique identifier)
- name: string (name of the learning path)
- description: string (description of the learning path)
- targetAudience: enum (student, researcher, engineer)
- skillLevel: enum (beginner, intermediate, advanced)
- moduleSequence: array of Module IDs (ordered list of modules in the path)
- estimatedDuration: integer (total estimated time in hours)
- createdAt: datetime
- updatedAt: datetime

**Relationships**:
- Contains many Course Modules (ordered)
- Assigned to many User Profiles
- Contains many Assessments

**Validation Rules**:
- Name is required and must be 5-50 characters
- Target audience must be one of the enum values
- Skill level must be one of the enum values
- Module sequence must not be empty and must reference valid modules

### Interactive Resource
**Definition**: Represents code examples, simulations, and exercises that users can interact with

**Attributes**:
- id: string (unique identifier)
- title: string (resource title)
- type: enum (code-example, simulation, exercise, project, video, diagram)
- content: string (the actual resource content or link)
- moduleId: string (ID of the module this resource belongs to)
- language: string (language code)
- tags: array of strings (tags for categorization)
- difficulty: enum (beginner, intermediate, advanced)
- estimatedTime: integer (time to complete in minutes)
- createdAt: datetime
- updatedAt: datetime

**Relationships**:
- Belongs to one Course Module
- Associated with many User Interactions

**Validation Rules**:
- Title is required and must be 5-100 characters
- Type must be one of the enum values
- Module ID must reference a valid module
- Language must be supported
- Estimated time must be positive

### Assessment
**Definition**: Represents quizzes, exercises, and practical assignments to validate user understanding

**Attributes**:
- id: string (unique identifier)
- title: string (assessment title)
- type: enum (quiz, practical-exercise, project, capstone)
- moduleId: string (ID of the module this assessment belongs to)
- questions: array of objects (question objects with text, type, options, etc.)
- passingScore: integer (minimum score required to pass, 0-100)
- timeLimit: integer (time limit in minutes, null if no limit)
- weight: float (weight of assessment in overall course grade, 0-1)
- createdAt: datetime
- updatedAt: datetime

**Relationships**:
- Belongs to one Course Module
- Submitted by many User Profiles
- Contains many Assessment Submissions

**Validation Rules**:
- Title is required and must be 5-100 characters
- Module ID must reference a valid module
- Questions array must not be empty
- Passing score must be between 0-100
- Weight must be between 0-1

### Assessment Submission
**Definition**: Represents a user's submission of an assessment

**Attributes**:
- id: string (unique identifier)
- userId: string (ID of the user submitting)
- assessmentId: string (ID of the assessment being submitted)
- answers: array of objects (user's answers to questions)
- score: integer (score received, 0-100)
- completedAt: datetime (when submission was completed)
- feedback: string (automated feedback provided)

**Relationships**:
- Submitted by one User Profile
- For one Assessment
- Contains many Answer Records

**Validation Rules**:
- User ID must reference a valid user
- Assessment ID must reference a valid assessment
- Score must be between 0-100
- Must be completed before submission

### Physical AI Concept
**Definition**: Represents principles of embodied intelligence and physical law understanding in AI systems

**Attributes**:
- id: string (unique identifier)
- name: string (name of the concept)
- description: string (detailed description)
- applications: array of strings (practical applications)
- relatedModules: array of Module IDs (modules where this concept is taught)
- prerequisites: array of Concept IDs (prerequisites for understanding)
- createdAt: datetime
- updatedAt: datetime

**Relationships**:
- Related to many Course Modules
- Prerequisite to many other Physical AI Concepts
- Covered in many Interactive Resources

**Validation Rules**:
- Name is required and must be 5-100 characters
- Description is required
- Related modules must reference valid modules

### ROS 2 Component
**Definition**: Represents nodes, topics, services, and actions in the Robot Operating System

**Attributes**:
- id: string (unique identifier)
- name: string (name of the component)
- type: enum (node, topic, service, action, package)
- description: string (detailed description)
- usageExamples: array of strings (code examples showing usage)
- relatedModules: array of Module IDs (modules where this component is taught)
- complexity: enum (basic, intermediate, advanced)
- createdAt: datetime
- updatedAt: datetime

**Relationships**:
- Related to many Course Modules
- Used in many Interactive Resources
- Connected to many Simulation Environments

**Validation Rules**:
- Name is required and must be 5-100 characters
- Type must be one of the enum values
- Description is required
- Complexity must be one of the enum values

### Simulation Environment
**Definition**: Represents Gazebo and Unity environments for robot testing

**Attributes**:
- id: string (unique identifier)
- name: string (name of the environment)
- type: enum (gazebo, unity, custom)
- description: string (detailed description)
- setupInstructions: string (instructions for setting up the environment)
- relatedModules: array of Module IDs (modules where this environment is used)
- requiredHardware: string (minimum hardware requirements)
- createdAt: datetime
- updatedAt: datetime

**Relationships**:
- Related to many Course Modules
- Used in many Interactive Resources
- Connected to many ROS 2 Components

**Validation Rules**:
- Name is required and must be 5-100 characters
- Type must be one of the enum values
- Description is required
- Related modules must reference valid modules

### NVIDIA Isaac Element
**Definition**: Represents components of the NVIDIA Isaac AI robot platform

**Attributes**:
- id: string (unique identifier)
- name: string (name of the element)
- category: enum (sdk, sim, perception, manipulation, navigation)
- description: string (detailed description)
- usageExamples: array of strings (code examples showing usage)
- relatedModules: array of Module IDs (modules where this element is taught)
- prerequisites: array of Isaac Element IDs (prerequisites)
- createdAt: datetime
- updatedAt: datetime

**Relationships**:
- Related to many Course Modules
- Prerequisite to many other Isaac Elements
- Used in many Interactive Resources

**Validation Rules**:
- Name is required and must be 5-100 characters
- Category must be one of the enum values
- Description is required
- Related modules must reference valid modules

### Humanoid Robot Design
**Definition**: Represents kinematics, dynamics, and interaction design elements for humanoid robots

**Attributes**:
- id: string (unique identifier)
- name: string (name of the design element)
- category: enum (kinematics, dynamics, interaction, control, locomotion)
- description: string (detailed description)
- applications: array of strings (practical applications)
- relatedModules: array of Module IDs (modules where this element is taught)
- complexity: enum (basic, intermediate, advanced)
- createdAt: datetime
- updatedAt: datetime

**Relationships**:
- Related to many Course Modules
- Used in many Interactive Resources
- Connected to many Physical AI Concepts

**Validation Rules**:
- Name is required and must be 5-100 characters
- Category must be one of the enum values
- Description is required
- Related modules must reference valid modules

### Conversational AI System
**Definition**: Represents GPT-based natural language processing and interaction capabilities

**Attributes**:
- id: string (unique identifier)
- name: string (name of the system component)
- category: enum (nlp, speech-recognition, dialogue-management, multimodal)
- description: string (detailed description)
- implementationExamples: array of strings (code examples showing implementation)
- relatedModules: array of Module IDs (modules where this system is taught)
- complexity: enum (basic, intermediate, advanced)
- createdAt: datetime
- updatedAt: datetime

**Relationships**:
- Related to many Course Modules
- Used in many Interactive Resources
- Connected to many Simulation Environments

**Validation Rules**:
- Name is required and must be 5-100 characters
- Category must be one of the enum values
- Description is required
- Related modules must reference valid modules

## Relationships

### Module Relationships
- Course Module contains many Interactive Resources
- Course Module contains many Assessments
- Course Module covers many Physical AI Concepts
- Course Module implements many ROS 2 Components
- Course Module integrates with many Simulation Environments
- Course Module utilizes many NVIDIA Isaac Elements
- Course Module implements many Humanoid Robot Design elements
- Course Module implements many Conversational AI Systems

### User Relationships
- User Profile enrolls in many Learning Paths
- User Profile completes many Course Modules
- User Profile submits many Assessment Submissions
- User Profile accesses many Interactive Resources

### Learning Path Relationships
- Learning Path contains many Course Modules (ordered)
- Learning Path targets specific User Profile roles
- Learning Path includes specific Physical AI Concepts
- Learning Path incorporates specific ROS 2 Components

### Assessment Relationships
- Assessment belongs to specific Course Module
- Assessment receives many Assessment Submissions from Users
- Assessment evaluates specific Learning Objectives

## State Transitions

### Module Completion States
- Not Started → In Progress → Completed
- Transitions triggered by user activity and assessment completion

### Assessment States
- Available → In Progress → Submitted → Graded → Reviewed
- Transitions based on user actions and automated grading

### User Progress States
- New User → Exploring → Enrolled → Active Learning → Assessment → Completed
- Transitions based on user engagement and milestone achievements

## Validation Rules

### Business Rules
1. A user cannot submit an assessment without completing the prerequisite modules
2. Assessment passing scores must be consistent with module difficulty levels
3. Learning paths must form a coherent progression of concepts
4. Interactive resources must be appropriate for the module's difficulty level
5. Course modules must be completed in sequence according to prerequisites

### Data Integrity Rules
1. All foreign key references must point to existing entities
2. Dates must be in valid ISO 8601 format
3. Numeric values must be within specified ranges
4. Text fields must not exceed maximum character limits
5. Email addresses must follow RFC 5322 standards

## Indexes
- User Profile: email (unique), role, skillLevel
- Course Module: week, difficulty, language
- Assessment: moduleId, type, createdAt
- Learning Path: targetAudience, skillLevel, estimatedDuration
- Interactive Resource: moduleId, type, difficulty
- Physical AI Concept: relatedModules
- ROS 2 Component: relatedModules, type
- Simulation Environment: relatedModules, type
- NVIDIA Isaac Element: relatedModules, category
- Humanoid Robot Design: relatedModules, category
- Conversational AI System: relatedModules, category

## Performance Considerations

### Query Optimization
- Frequently accessed entities should have appropriate indexes
- Assessment results should be aggregated for performance
- User progress tracking should be optimized for frequent updates

### Storage Optimization
- Large content (videos, simulations) should be stored externally
- Static content should be cached aggressively
- User interaction data should be archived periodically

## Security Considerations

### Data Protection
- User profile data must be encrypted at rest
- Assessment submissions must be protected from tampering
- Authentication required for progress tracking
- Privacy controls for user data sharing

### Access Control
- Module access may be restricted based on learning path
- Assessment attempts may be limited to prevent cheating
- Administrative access required for content modification
- Role-based access for different user types

## Extensibility Points

### Future Enhancements
- Additional language support (currently only English)
- Advanced analytics and reporting
- Social learning features
- Integration with external LMS systems
- Advanced simulation environments
- Extended reality (XR) integration
- Advanced AI tutoring systems