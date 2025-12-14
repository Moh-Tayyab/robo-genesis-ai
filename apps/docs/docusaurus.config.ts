import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'RoboGenesis AI: Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive textbook for learning robotics with AI-native approaches',
  favicon: 'img/favicon.ico',

  // GitHub Pages deployment configuration
  url: 'https://Moh-Tayyab.github.io',
  baseUrl: '/robo-genesis-ai/',

  organizationName: 'Moh-Tayyab',
  projectName: 'robo-genesis-ai',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'warn',

  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: '/',
          sidebarPath: './sidebars.ts',
          editUrl: 'https://github.com/zeeshan080/robo-genesis-ai/tree/development/apps/docs/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'RoboGenesis AI',
      logo: {
        alt: 'RoboGenesis AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'mainSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          to: '/profile',
          label: 'Profile',
          position: 'right',
        },
        {
          href: 'https://github.com/Moh-Tayyab/robo-genesis-ai',
          label: 'GitHub',
          position: 'right',
        },
        {
          type: 'custom-navbarAuthButton',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Documentation',
          items: [
            {
              label: 'Textbook',
              to: '/',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/Moh-Tayyab/robo-genesis-ai',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} RoboGenesis AI. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'json'],
    },
    // Local search configuration
    algolia: undefined, // Disable Algolia
  } satisfies Preset.ThemeConfig,

  plugins: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        language: ['en'],
        indexDocs: true,
        indexBlog: false,
        docsRouteBasePath: '/',
      },
    ],
  ],

  customFields: {
    // ChatKit backend API URL (can be overridden via env var)
    chatkitApiUrl: process.env.CHATKIT_API_URL || 'http://localhost:8000',
  },
};

export default config;
