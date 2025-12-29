import React from 'react';
import { CourseModule } from '../models/course_module';
import './TextbookNavigation.css';

interface TextbookNavigationProps {
  modules: CourseModule[];
  currentModuleId?: string;
  currentSectionId?: string;
  onModuleSelect: (moduleId: string) => void;
  onSectionSelect: (moduleId: string, sectionId: string) => void;
}

const TextbookNavigation: React.FC<TextbookNavigationProps> = ({
  modules,
  currentModuleId,
  currentSectionId,
  onModuleSelect,
  onSectionSelect
}) => {
  return (
    <nav className="textbook-navigation">
      <h3>Textbook Contents</h3>
      <ul className="module-list">
        {modules.map(module => (
          <li
            key={module.moduleId}
            className={`module-item ${currentModuleId === module.moduleId ? 'active-module' : ''}`}
          >
            <button
              className={`module-button ${currentModuleId === module.moduleId ? 'active' : ''}`}
              onClick={() => onModuleSelect(module.moduleId)}
            >
              {module.title}
              <span className="week-range">({module.weekRange})</span>
            </button>

            {currentModuleId === module.moduleId && module.sections && (
              <ul className="section-list">
                {module.sections.map(section => (
                  <li
                    key={section.id}
                    className={`section-item ${currentSectionId === section.id ? 'active-section' : ''}`}
                  >
                    <button
                      className={`section-button ${currentSectionId === section.id ? 'active' : ''}`}
                      onClick={() => onSectionSelect(module.moduleId, section.id)}
                    >
                      {section.title}
                    </button>
                  </li>
                ))}
              </ul>
            )}
          </li>
        ))}
      </ul>
    </nav>
  );
};

export default TextbookNavigation;