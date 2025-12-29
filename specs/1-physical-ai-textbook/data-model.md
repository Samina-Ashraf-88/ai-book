# Data Model: Physical AI & Humanoid Robotics Textbook

## User Entity
- **userId**: Unique identifier for each user
- **email**: User's email address (required for authentication)
- **softwareBackground**: Enum (None/Beginner/Intermediate/Advanced)
- **aiExperience**: Enum (None/Basic ML/Deep Learning/Production AI)
- **roboticsExperience**: Enum (None/Hobby Projects/Academic/Professional)
- **programmingLanguages**: Array of strings (Python, C++, JavaScript, etc.)
- **hardwareExperience**: Enum (None/Raspberry Pi/Arduino/Jetson/Custom Boards)
- **learningStyle**: Enum (Visual/Hands-on/Theoretical/Mixed)
- **createdAt**: Timestamp for account creation
- **lastLogin**: Timestamp for last login
- **progress**: Object tracking completion status for each module

## Course Module Entity
- **moduleId**: Unique identifier for each module
- **title**: Title of the module (e.g., "ROS 2 Fundamentals")
- **description**: Brief description of the module content
- **weekRange**: Range of weeks (e.g., "Weeks 3-5")
- **content**: Markdown content for the module
- **learningObjectives**: Array of learning objectives
- **prerequisites**: Array of prerequisite modules or knowledge

## Assessment Entity
- **assessmentId**: Unique identifier for each assessment
- **moduleId**: Reference to the associated module
- **title**: Title of the assessment
- **questions**: Array of question objects with type, content, and answers
- **maxScore**: Maximum possible score
- **timeLimit**: Time limit for completion (optional)

## Progress Record Entity
- **progressId**: Unique identifier for each progress record
- **userId**: Reference to the user
- **moduleId**: Reference to the module
- **completed**: Boolean indicating completion status
- **score**: Score achieved on associated assessment (if applicable)
- **timeSpent**: Time spent on the module in minutes
- **lastAccessed**: Timestamp of last access
- **completedSections**: Array of completed section identifiers

## Hardware Guide Entity
- **guideId**: Unique identifier for each guide
- **title**: Title of the hardware guide
- **category**: Category (e.g., "Workstation", "Edge Kit", "Robot Options")
- **content**: Markdown content with specifications and setup instructions
- **targetAudience**: Enum (Budget/Standard/Premium)
- **estimatedCost**: Estimated cost range
- **difficultyLevel**: Enum (Beginner/Intermediate/Advanced)