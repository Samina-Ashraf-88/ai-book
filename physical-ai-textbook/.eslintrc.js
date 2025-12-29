module.exports = {
  extends: [
    '@docusaurus/eslint-plugin',
    'prettier',
    'plugin:prettier/recommended',
  ],
  plugins: ['prettier'],
  rules: {
    'prettier/prettier': 'error',
  },
  env: {
    browser: true,
    node: true,
  },
};