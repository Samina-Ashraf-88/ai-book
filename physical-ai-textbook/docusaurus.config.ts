import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Bridging the gap between digital AI and physical embodied intelligence',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: process.env.DEPLOYMENT_URL || process.env.VERCEL_URL || 'https://physical-ai-textbook.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel deployment, use root path
  baseUrl: '/',

  // Deployment config for Vercel
  organizationName: 'Samina-Ashraf-88', // Your GitHub org/user name.
  projectName: 'Hackathon-1', // Your repo name.

  onBrokenLinks: process.env.NODE_ENV === 'production' ? 'warn' : 'throw',
  onBrokenMarkdownLinks: 'warn',
  markdown: {
    mermaid: true,
    parseFrontMatter: (params) => {
      // Use the default parsing behavior
      return params.defaultParseFrontMatter(params);
    },
  },

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: ({versionDocsDirPath, docPath}) => {
            // Use relative path for local development and GitHub for production
            if (process.env.NODE_ENV === 'development' || process.env.VERCEL_ENV === 'development') {
              return undefined; // Disable edit URL in development
            }
            return `https://github.com/Samina-Ashraf-88/Hackathon-1/edit/main/physical-ai-textbook${versionDocsDirPath}/${docPath}`;
          },
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl: ({versionDocsDirPath, docPath}) => {
            // Use relative path for local development and GitHub for production
            if (process.env.NODE_ENV === 'development' || process.env.VERCEL_ENV === 'development') {
              return undefined; // Disable edit URL in development
            }
            return `https://github.com/Samina-Ashraf-88/Hackathon-1/edit/main/physical-ai-textbook${versionDocsDirPath}/${docPath}`;
          },
          // Useful options to enforce blogging best practices
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/Samina-Ashraf-88/Hackathon-1',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Course',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Hardware Requirements',
              to: '/docs/hardware-guide/workstation',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'Course Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Hardware Guide',
              to: '/docs/hardware-guide/workstation',
            },
            {
              label: 'Capstone Project',
              to: '/docs/capstone-project',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Samina-Ashraf-88/Hackathon-1',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
