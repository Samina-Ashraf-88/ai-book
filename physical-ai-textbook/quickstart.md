# Quickstart Guide for Physical AI & Humanoid Robotics Textbook

This guide will help you get started with the Physical AI & Humanoid Robotics textbook application quickly.

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn package manager
- Git version control system
- A modern web browser

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd physical-ai-textbook
```

2. Install dependencies:
```bash
npm install
```

3. Create a `.env` file in the root directory with the following content:
```env
NODE_ENV=development
```

## Running the Application

### Development Mode

To run the application in development mode with hot reloading:

```bash
npm start
```

The application will be available at `http://localhost:3000`.

### Production Mode

To build and run the application in production mode:

```bash
npm run build
npm run serve
```

The application will be available at `http://localhost:3000`.

## Key Features

### User Registration & Personalization
- Register with background information about your experience
- Get personalized content based on your skill level
- Access tailored learning paths

### Interactive Textbook Content
- Navigate through modules on ROS 2, Gazebo, NVIDIA Isaac, and VLA
- Access interactive code examples
- Engage with visual diagrams and illustrations

### Content Translation
- Toggle between English and Urdu
- Right-to-left (RTL) support for Urdu
- Technical term translations

### Assessment & Progress Tracking
- Complete module assessments
- Track your progress through the course
- View detailed progress statistics

### Hardware Guides
- Access hardware requirements guides
- Choose from workstation, edge kit, or robot lab options
- Compare cloud vs on-premise lab options

## Project Structure

```
physical-ai-textbook/
├── docs/                 # Textbook content
├── src/                  # Source code
│   ├── components/       # React components
│   ├── models/           # Data models
│   ├── services/         # Service implementations
│   ├── utils/            # Utility functions
│   └── i18n/             # Internationalization files
├── static/               # Static assets
├── tests/                # Test files
└── package.json          # Project configuration
```

## Development

### Adding New Content
1. Add new content to the `docs/` directory using Markdown
2. Update the sidebar configuration in `sidebars.ts`
3. Test the new content in development mode

### Adding New Components
1. Create new components in `src/components/`
2. Follow the existing naming and structure conventions
3. Implement proper internationalization support
4. Add appropriate tests

## Troubleshooting

### Common Issues

**Issue**: Application fails to start
**Solution**: Ensure all dependencies are installed with `npm install`

**Issue**: Content doesn't display properly
**Solution**: Check that Markdown files are properly formatted

**Issue**: Translation not working
**Solution**: Verify that translation files exist in `i18n/` directory

## Next Steps

After completing this quickstart:

1. Explore the textbook content in the `docs/` directory
2. Register for an account to access personalized features
3. Begin with Module 1: ROS 2 Fundamentals
4. Complete the assessments to track your progress