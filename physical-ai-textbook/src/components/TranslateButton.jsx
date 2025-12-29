import React, { useState, useEffect } from 'react';
import { useTranslation } from 'react-i18next';
import './TranslateButton.css';

/**
 * TranslateButton Component
 * Provides a toggle button for switching between English and Urdu languages
 */
const TranslateButton = ({ className = '', size = 'medium' }) => {
  const { i18n, t } = useTranslation();
  const [currentLanguage, setCurrentLanguage] = useState(i18n.language || 'en');
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);

  // Update local state when language changes externally
  useEffect(() => {
    const handleLanguageChange = () => {
      setCurrentLanguage(i18n.language);
    };

    i18n.on('languageChanged', handleLanguageChange);
    return () => {
      i18n.off('languageChanged', handleLanguageChange);
    };
  }, [i18n]);

  const toggleLanguage = async (langCode) => {
    if (langCode !== currentLanguage) {
      try {
        await i18n.changeLanguage(langCode);
        setCurrentLanguage(langCode);
        setIsDropdownOpen(false);
      } catch (error) {
        console.error('Error changing language:', error);
      }
    }
  };

  const toggleDropdown = () => {
    setIsDropdownOpen(!isDropdownOpen);
  };

  // Get display text based on current language
  const getLanguageDisplay = (langCode) => {
    if (langCode === 'ur') {
      return 'اردو';
    } else if (langCode === 'en') {
      return 'English';
    }
    return langCode.toUpperCase();
  };

  // Get flag emoji based on language
  const getLanguageFlag = (langCode) => {
    switch (langCode) {
      case 'ur':
        return '🇵🇰'; // Pakistan flag for Urdu
      case 'en':
        return '🇬🇧'; // UK flag for English
      default:
        return '🌐';
    }
  };

  // Available languages
  const availableLanguages = [
    { code: 'en', name: 'English', flag: getLanguageFlag('en') },
    { code: 'ur', name: 'اردو', flag: getLanguageFlag('ur') }
  ];

  const currentLangInfo = availableLanguages.find(lang => lang.code === currentLanguage) || availableLanguages[0];

  return (
    <div className={`translate-button-container ${className}`}>
      <button
        className={`translate-toggle-button ${size} ${isDropdownOpen ? 'active' : ''}`}
        onClick={toggleDropdown}
        aria-label={t('toggleLanguage', 'Toggle Language')}
        aria-expanded={isDropdownOpen}
        aria-haspopup="listbox"
      >
        <span className="language-flag">{currentLangInfo.flag}</span>
        <span className="language-text">{getLanguageDisplay(currentLanguage)}</span>
        <span className={`dropdown-arrow ${isDropdownOpen ? 'rotated' : ''}`}>▼</span>
      </button>

      {isDropdownOpen && (
        <div className="language-dropdown">
          <ul
            role="listbox"
            className="language-list"
            aria-label={t('language', 'Available Languages')}
          >
            {availableLanguages
              .filter(lang => lang.code !== currentLanguage)
              .map((lang) => (
                <li key={lang.code} className="language-option">
                  <button
                    className="language-option-button"
                    onClick={() => toggleLanguage(lang.code)}
                    aria-label={`${t('switchTo', 'Switch to')} ${lang.name}`}
                    role="option"
                    aria-selected={currentLanguage === lang.code}
                  >
                    <span className="language-flag">{lang.flag}</span>
                    <span className="language-name">{lang.name}</span>
                  </button>
                </li>
              ))}
          </ul>
        </div>
      )}
    </div>
  );
};

export default TranslateButton;