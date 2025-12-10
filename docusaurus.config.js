// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers to understand what
// you are autiting in the file and check against your expectations.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'AI in Motion: Foundations of Physical AI and Humanoid Robotics',
  tagline: 'Learn Physical AI and Humanoid Robotics through hands-on practice',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/ai-in-motion',

  // GitHub pages deployment config.
  organizationName: 'your-username', // Usually your GitHub org/user name.
  projectName: 'ai-in-motion', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-username/ai-in-motion/tree/main/',
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-username/ai-in-motion/tree/main/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  plugins: [
    // Plugin to load our translation script
    async function translatePlugin(context, options) {
      return {
        name: 'translate-plugin',
        injectHtmlTags() {
          return {
            postBodyTags: [
              '<script src="/js/translate.js"></script>'
            ],
          };
        },
      };
    },
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'AI in Motion',
        logo: {
          alt: 'AI in Motion Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Tutorial',
          },
          {to: '/blog', label: 'Blog', position: 'left'},
          {
            href: 'https://github.com/your-username/ai-in-motion',
            label: 'GitHub',
            position: 'right',
          },
          {
            type: 'html',
            position: 'right',
            value: '<button id="translateBtn" onclick="window.translateToUrdu && window.translateToUrdu()" style="background-color: #4285f4; color: white; border: none; padding: 0.5rem 1rem; border-radius: 24px; cursor: pointer; box-shadow: 0 2px 4px rgba(0,0,0,0.1); margin-left: 0.5rem; font-size: 0.8rem;">Translate to Urdu</button>',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Tutorial',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/ai-in-motion',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/ai-in-motion',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Blog',
                to: '/blog',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/your-username/ai-in-motion',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} AI in Motion. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer/themes/github'),
        darkTheme: require('prism-react-renderer/themes/dracula'),
      },
    }),
};

module.exports = config;