import React, { useEffect } from 'react';
import { useTranslation } from 'react-i18next';
import { CourseModule, ModuleSection } from '../models/course_module';
import InteractiveCode from './InteractiveCode';
import './ContentDisplay.css';

interface ContentDisplayProps {
  module: CourseModule;
  sectionId?: string;
  onContentUpdate?: (content: string) => void;
}

const ContentDisplay: React.FC<ContentDisplayProps> = ({
  module,
  sectionId,
  onContentUpdate
}) => {
  const { t } = useTranslation(['common', 'textbook']);

  // Find the section to display, or default to the first section
  const section = module.sections.find(s => s.id === sectionId) || module.sections[0];

  // Initialize syntax highlighting when component mounts
  useEffect(() => {
    // In a real implementation, we would initialize syntax highlighting here
    // For example, with Prism.js or a similar library
    if (typeof window !== 'undefined') {
      // Dynamically import syntax highlighting library if needed
      // import('prismjs').then(Prism => {
      //   Prism.highlightAll();
      // });
    }
  }, []);

  // Import DOMPurify for sanitization (this would be installed as a dependency)
  // For now, we'll implement a basic sanitization function
  const sanitizeHTML = (html: string): string => {
    // Basic sanitization to remove potentially dangerous elements
    // In a real implementation, use DOMPurify or similar library
    return html
      .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '') // Remove script tags
      .replace(/<iframe\b[^<]*(?:(?!<\/iframe>)<[^<]*)*<\/iframe>/gi, '') // Remove iframe tags
      .replace(/<object\b[^<]*(?:(?!<\/object>)<[^<]*)*<\/object>/gi, '') // Remove object tags
      .replace(/<embed\b[^<]*(?:(?!<\/embed>)<[^<]*)*<\/embed>/gi, '') // Remove embed tags
      .replace(/<form\b[^<]*(?:(?!<\/form>)<[^<]*)*<\/form>/gi, '') // Remove form tags
      .replace(/(on\w+="[^"]*")/gi, '') // Remove event handlers
      .replace(/javascript:/gi, '') // Remove javascript: protocols
      .replace(/data:/gi, '') // Remove data: protocols (potential XSS)
      .replace(/vbscript:/gi, ''); // Remove vbscript: protocols
  };

  // Render content with proper formatting
  const renderContent = (content: string) => {
    // In a real implementation, this would parse the content and render
    // interactive elements like code blocks, diagrams, etc.
    // For security, we sanitize the HTML content
    const sanitizedContent = sanitizeHTML(content);

    return (
      <div
        className="content-body"
        dangerouslySetInnerHTML={{ __html: sanitizedContent }}
      />
    );
  };

  // Render interactive elements based on type
  const renderInteractiveElement = (element: any, index: number) => {
    switch (element.type) {
      case 'code-example':
        return (
          <InteractiveCode
            key={index}
            code={element.content}
            language={element.language}
            caption={element.caption}
            copyable={element.copyable}
          />
        );
      case 'diagram':
        return (
          <div key={index} className="diagram-container">
            <img src={element.content} alt={element.caption} />
            {element.caption && <div className="diagram-caption">{element.caption}</div>}
          </div>
        );
      case 'exercise':
        return (
          <div key={index} className="exercise-container">
            <h4>Exercise: {element.title || 'Practice'}</h4>
            <div>{element.question}</div>
            {element.options && (
              <div className="exercise-options">
                {element.options.map((option: string, optIndex: number) => (
                  <div key={optIndex} className="exercise-option">
                    <input type="radio" id={`opt-${index}-${optIndex}`} name={`exercise-${index}`} />
                    <label htmlFor={`opt-${index}-${optIndex}`}>{option}</label>
                  </div>
                ))}
              </div>
            )}
          </div>
        );
      default:
        return (
          <div key={index} className="interactive-element">
            <p>Interactive element: {element.type}</p>
          </div>
        );
    }
  };

  return (
    <div className="content-display">
      <header className="content-header">
        <h1>{t(module.titleKey || module.title, { defaultValue: module.title })}</h1>
        {section && <h2>{t(section.titleKey || section.title, { defaultValue: section.title })}</h2>}
      </header>

      <div className="content-main">
        {section ? (
          <>
            {renderContent(section.content)}

            {section.interactiveElements && section.interactiveElements.map((element, index) =>
              renderInteractiveElement(element, index)
            )}
          </>
        ) : (
          <div className="no-section">
            <p>{t('noSectionSelected', { defaultValue: 'No section selected. Please select a section from the navigation.' })}</p>
          </div>
        )}
      </div>

      <footer className="content-footer">
        <div className="learning-objectives">
          <h4>{t('learningObjectives', { defaultValue: 'Learning Objectives' })}</h4>
          <ul>
            {module.learningObjectives.map((obj, index) => (
              <li key={index}>{t(obj.key || obj, { defaultValue: obj })}</li>
            ))}
          </ul>
        </div>

        {section?.objectives && section.objectives.length > 0 && (
          <div className="section-objectives">
            <h4>{t('sectionObjectives', { defaultValue: 'Section Objectives' })}</h4>
            <ul>
              {section.objectives.map((obj, index) => (
                <li key={index}>{t(obj.key || obj, { defaultValue: obj })}</li>
              ))}
            </ul>
          </div>
        )}
      </footer>
    </div>
  );
};

export default ContentDisplay;