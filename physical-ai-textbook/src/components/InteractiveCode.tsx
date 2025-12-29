import React, { useState } from 'react';
import './InteractiveCode.css';

interface InteractiveCodeProps {
  code: string;
  language?: string;
  caption?: string;
  copyable?: boolean;
  showLineNumbers?: boolean;
  editable?: boolean;
  onCodeChange?: (newCode: string) => void;
}

const InteractiveCode: React.FC<InteractiveCodeProps> = ({
  code,
  language = 'python',
  caption,
  copyable = true,
  showLineNumbers = true,
  editable = false,
  onCodeChange
}) => {
  const [codeContent, setCodeContent] = useState(code);
  const [copied, setCopied] = useState(false);
  const [isEditing, setIsEditing] = useState(false);

  const handleCopy = () => {
    navigator.clipboard.writeText(codeContent)
      .then(() => {
        setCopied(true);
        setTimeout(() => setCopied(false), 2000); // Reset after 2 seconds
      })
      .catch(err => {
        console.error('Failed to copy code: ', err);
      });
  };

  const handleEdit = () => {
    if (editable) {
      setIsEditing(true);
    }
  };

  const handleSave = () => {
    if (onCodeChange) {
      onCodeChange(codeContent);
    }
    setIsEditing(false);
  };

  const handleCancel = () => {
    setCodeContent(code); // Reset to original code
    setIsEditing(false);
  };

  const handleCodeChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    setCodeContent(e.target.value);
  };

  // Format code with line numbers
  const codeLines = codeContent.split('\n');
  const numberedCode = codeLines.map((line, index) => (
    <div key={index} className="code-line">
      {showLineNumbers && <span className="line-number">{index + 1}</span>}
      <span className="line-content">{line}</span>
    </div>
  ));

  return (
    <div className="interactive-code-container">
      {caption && <div className="code-caption">{caption}</div>}
      <div className="code-header">
        <div className="code-language">{language}</div>
        <div className="code-actions">
          {copyable && (
            <button className="code-action-btn" onClick={handleCopy} title="Copy code">
              {copied ? '✓ Copied!' : 'Copy'}
            </button>
          )}
          {editable && !isEditing && (
            <button className="code-action-btn" onClick={handleEdit} title="Edit code">
              Edit
            </button>
          )}
          {editable && isEditing && (
            <>
              <button className="code-action-btn" onClick={handleSave} title="Save changes">
                Save
              </button>
              <button className="code-action-btn" onClick={handleCancel} title="Cancel changes">
                Cancel
              </button>
            </>
          )}
        </div>
      </div>
      {isEditing ? (
        <textarea
          className="code-editor"
          value={codeContent}
          onChange={handleCodeChange}
          spellCheck="false"
        />
      ) : (
        <pre className={`code-block language-${language}`}>
          <code className={`language-${language}`}>
            {numberedCode}
          </code>
        </pre>
      )}
    </div>
  );
};

export default InteractiveCode;