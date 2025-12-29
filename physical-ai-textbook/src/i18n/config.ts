import i18n from 'i18next';
import { initReactI18next } from 'react-i18next';
import LanguageDetector from 'i18next-browser-languagedetector';
import Backend from 'i18next-http-backend';

i18n
  .use(Backend)
  .use(LanguageDetector)
  .use(initReactI18next)
  .init({
    // Load translations from JSON files in the i18n directory
    backend: {
      loadPath: '/i18n/{{lng}}/{{ns}}.json', // Path to load translation files
      addPath: '/i18n/{{lng}}/{{ns}}', // Path to add new translations
    },

    // Namespaces - these correspond to your JSON files
    ns: ['common', 'textbook', 'module1', 'capstone'],
    defaultNS: 'common',

    fallbackLng: 'en',
    debug: true,

    interpolation: {
      escapeValue: false, // not needed for react as it escapes by default
    },

    // Language detection options
    detection: {
      order: ['localStorage', 'navigator'],
      caches: ['localStorage'],
    },

    // Enable backend for loading translation files
    useSuspense: false, // Set to false to avoid suspense issues
  });

// Set RTL attribute on HTML element when language changes
i18n.on('languageChanged', (lng) => {
  const htmlTag = document.documentElement;
  if (lng === 'ur') {
    htmlTag.setAttribute('dir', 'rtl');
    htmlTag.setAttribute('lang', 'ur');
  } else {
    htmlTag.setAttribute('dir', 'ltr');
    htmlTag.setAttribute('lang', 'en');
  }
});

export default i18n;