# Quickstart Guide: Physical_Humanoid_AI_Robotics_Course

## Overview

This quickstart guide provides a fast path to getting up and running with the Physical_Humanoid_AI_Robotics_Course platform. It covers the essential steps to set up the development environment, understand the project structure, and begin contributing to the course content.

## Prerequisites

Before starting with the Physical_Humanoid_AI_Robotics_Course development, ensure you have the following installed:

### System Requirements
- **Operating System**: Windows 10+, macOS 10.14+, or Ubuntu 18.04+
- **Node.js**: Version 18.0 or higher
- **npm**: Version 8.0 or higher (usually comes with Node.js)
- **Git**: Version 2.20 or higher
- **Python**: Version 3.8 or higher (for certain tools)
- **Memory**: 8GB RAM minimum (16GB+ recommended)
- **Disk Space**: 5GB+ available space

### Development Tools
- **Code Editor**: VS Code recommended (with Docusaurus extensions)
- **Terminal**: Command line interface (PowerShell, bash, or similar)
- **Browser**: Chrome, Firefox, or Edge (latest versions)

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/your-org/physical-humanoid-ai-robotics-course.git
cd physical-humanoid-ai-robotics-course
```

### 2. Navigate to Website Directory

```bash
cd website
```

### 3. Install Dependencies

```bash
npm install
```

This will install all necessary dependencies for the Docusaurus-based course platform.

### 4. Verify Installation

```bash
npm run build
```

If the build completes successfully, your installation is ready.

## Development Server

### Start Local Development Server

```bash
npm start
```

This command will:
- Start a local development server
- Open your browser to http://localhost:3000
- Enable hot reloading for instant updates as you modify content

### Development Server Options

```bash
# Start with specific port
npm start -- --port 8080

# Start with host binding
npm start -- --host 0.0.0.0

# Start with open in browser disabled
npm start -- --no-open
```

## Project Structure

### Key Directories

```
website/
├── docusaurus.config.js    # Main Docusaurus configuration
├── package.json            # Project dependencies and scripts
├── static/                 # Static assets (images, documents)
├── src/
│   ├── components/         # Custom React components
│   ├── pages/              # Additional pages if needed
│   └── css/                # Custom styles
├── docs/                   # Course content organized by weeks
│   ├── intro.md            # Course introduction
│   ├── week-01-physical-ai/    # Week 1 content
│   ├── week-02-physical-ai/    # Week 2 content
│   ├── week-03-ros2/           # Week 3 content
│   ├── week-04-ros2/           # Week 4 content
│   ├── week-05-ros2/           # Week 5 content
│   ├── week-06-simulation/     # Week 6 content
│   ├── week-07-simulation/     # Week 7 content
│   ├── week-08-nvidia-isaac/   # Week 8 content
│   ├── week-09-nvidia-isaac/   # Week 9 content
│   ├── week-10-nvidia-isaac/   # Week 10 content
│   ├── week-11-humanoid-robotics/  # Week 11 content
│   ├── week-12-humanoid-robotics/  # Week 12 content
│   └── week-13-conversational-ai/  # Week 13 content
├── i18n/                   # Internationalization files (English)
└── tests/                  # Testing files
    ├── e2e/
    └── unit/
```

### Content Organization

Course content is organized by weeks in the `docs/` directory:

- **Week 1-2**: Physical AI and Embodied Intelligence
- **Week 3-5**: ROS 2 Framework and Applications
- **Week 6-7**: Simulation Environments (Gazebo and Unity)
- **Week 8-10**: NVIDIA Isaac AI Platform
- **Week 11-12**: Humanoid Robot Kinematics and Dynamics
- **Week 13**: Conversational AI for Robotics

## Creating New Content

### 1. Adding a New Module

To add a new module to an existing week:

```bash
# Create a new module file
touch docs/week-01-physical-ai/new-module.md
```

### 2. Module Template

Use this template for new modules:

```markdown
---
title: Module Title
sidebar_label: Module Title
description: Brief description of the module content
keywords: [keyword1, keyword2, keyword3]
---

## Learning Objectives

- Objective 1
- Objective 2
- Objective 3

## Introduction

Brief introduction to the module topic...

## Main Content

Detailed content for the module...

### Subsection

More detailed information...

## Summary

Brief summary of key points...

## Further Reading

- [Reference 1](link)
- [Reference 2](link)

## Assessment

Complete the assessment for this module to track your progress.
```

### 3. Adding a New Week

To add a new week to the course:

1. Create a new directory:
   ```bash
   mkdir docs/week-XX-topic-name
   ```

2. Add the week to the sidebar in `sidebars.js`:
   ```javascript
   {
     type: 'category',
     label: 'Week XX: Topic Name',
     items: [
       'week-XX-topic-name/module1',
       'week-XX-topic-name/module2',
       // Add more modules as needed
     ],
   },
   ```

## Custom Components

### Interactive Code Examples

Use the following component for interactive code examples:

```jsx
import CodeBlock from '@theme/CodeBlock';

<CodeBlock language="python">
{`# Example Python code
def example_function():
    print("Hello, Robotics!")
`}
</CodeBlock>
```

### Assessment Components

For creating assessments, use the custom Assessment component:

```jsx
import Assessment from '@site/src/components/Assessment';

<Assessment
  id="module-1-quiz"
  title="Module 1 Quiz"
  questions={[
    {
      id: 1,
      text: "What is Physical AI?",
      type: "multiple-choice",
      options: [
        { id: "a", text: "AI in physical environments" },
        { id: "b", text: "AI for digital systems only" },
        { id: "c", text: "AI without embodiment" }
      ],
      correctAnswer: "a"
    }
  ]}
/>
```

### Progress Tracking

Use the ProgressTracker component to track user progress:

```jsx
import ProgressTracker from '@site/src/components/ProgressTracker';

<ProgressTracker
  moduleId="week-01-module-1"
  title="Introduction to Physical AI"
/>
```

## Building for Production

### Build the Static Site

```bash
npm run build
```

This command generates a static website in the `build/` directory that can be deployed to any web server.

### Serve Built Site Locally

```bash
npm run serve
```

This serves the built site locally to test production build behavior.

### Build with Specific Configuration

```bash
# Build with different environment
NODE_ENV=production npm run build

# Build with specific output directory
npm run build -- --out-dir custom-build-folder
```

## Testing

### Unit Tests

```bash
npm test
```

### End-to-End Tests

```bash
npm run test:e2e
```

### Linting

```bash
npm run lint
```

### Type Checking

```bash
npm run typecheck
```

## Configuration

### Docusaurus Configuration

The main configuration is in `docusaurus.config.js`. Key settings include:

- `title`: Site title
- `tagline`: Site tagline
- `url`: Production URL
- `baseUrl`: Base URL for deployment
- `favicon`: Site favicon
- `organizationName`: GitHub org name (for deployment)
- `projectName`: GitHub project name (for deployment)

### Theme Configuration

Customize the theme in the `themeConfig` section:

```javascript
themeConfig: {
  navbar: {
    title: 'Course Title',
    logo: {
      alt: 'Logo',
      src: 'img/logo.svg',
    },
    items: [
      // Navigation items
    ],
  },
  footer: {
    style: 'dark',
    links: [
      // Footer links
    ],
    copyright: `Copyright © ${new Date().getFullYear()} Course Name`,
  },
},
```

## Deployment

### GitHub Pages Deployment

```bash
npm run deploy
```

### Custom Deployment

For custom deployment, build the site and upload the contents of the `build/` directory to your web server.

### Environment Variables

Set environment variables in a `.env` file:

```
# Google Analytics
GA_ID=your-google-analytics-id

# Algolia Search
ALGOLIA_API_KEY=your-algolia-api-key
ALGOLIA_INDEX_NAME=your-index-name

# Custom variables
CUSTOM_FEATURE_FLAG=true
```

## Troubleshooting

### Common Issues

#### 1. Port Already in Use
**Problem**: `Error: Port 3000 is already in use`
**Solution**: Use a different port with `npm start -- --port 3001`

#### 2. Dependency Installation Issues
**Problem**: `npm install` fails with permission errors
**Solution**: 
```bash
# Clear npm cache
npm cache clean --force

# Reinstall dependencies
rm -rf node_modules package-lock.json
npm install
```

#### 3. Build Errors
**Problem**: `npm run build` fails
**Solution**: Check for syntax errors in Markdown files and ensure all referenced files exist

#### 4. Hot Reload Not Working
**Problem**: Changes don't appear in browser automatically
**Solution**: 
- Restart the development server
- Clear browser cache
- Check for JavaScript errors in console

### Performance Tips

1. **Large Images**: Optimize images before adding to the site
2. **Heavy Components**: Use lazy loading for heavy components
3. **Markdown Files**: Keep individual files under 10KB for better performance
4. **External Resources**: Minimize external dependencies

## Contributing

### Adding New Content

1. Create the content file in the appropriate week directory
2. Add it to the sidebar in `sidebars.js`
3. Ensure it follows the module template
4. Test locally before committing

### Updating Components

1. Modify components in `src/components/`
2. Test functionality in development mode
3. Ensure backward compatibility
4. Update documentation if needed

### Code Style

- Use TypeScript for new components
- Follow React best practices
- Maintain consistent naming conventions
- Write clear, descriptive comments

## Advanced Features

### Search Configuration

Search is configured in `docusaurus.config.js`:

```javascript
presets: [
  [
    'classic',
    {
      docs: {
        sidebarPath: require.resolve('./sidebars.js'),
        editUrl: 'https://github.com/your-org/repo/edit/main/website/',
      },
      theme: {
        customCss: require.resolve('./src/css/custom.css'),
      },
    },
  ],
],
```

### Analytics Integration

Google Analytics and other analytics tools can be integrated through plugins in the configuration.

### Internationalization

Though currently configured for English only, the framework supports i18n:

```bash
# Add new translation
npm run write-translations -- --locale en
```

## Performance Optimization

### Bundle Analysis

```bash
npm run build
npm run serve
# Visit http://localhost:3000/__docusaurus/debug/bundles
```

### Image Optimization

- Use WebP format where possible
- Compress images before adding
- Use appropriate dimensions
- Implement lazy loading

## Next Steps

1. **Explore existing modules** to understand the content structure
2. **Create your first module** using the template
3. **Test your changes** in the development server
4. **Add to the sidebar** to make it navigable
5. **Submit a pull request** for review

For more detailed information about Docusaurus features, visit the [official documentation](https://docusaurus.io/docs).

## Support

- **Documentation**: Check the official Docusaurus docs
- **Community**: Join the Docusaurus Discord server
- **Issues**: Report issues on the GitHub repository
- **Questions**: Ask questions in the project's discussion forum