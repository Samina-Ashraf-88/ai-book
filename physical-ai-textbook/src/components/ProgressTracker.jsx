import React, { useState, useEffect } from 'react';
import './ProgressTracker.css';

/**
 * Progress Tracker Component
 * Displays user progress through the course modules
 */
const ProgressTracker = ({ userId, modules, onModuleSelect }) => {
  const [userProgress, setUserProgress] = useState({});
  const [selectedModule, setSelectedModule] = useState(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState(null);

  // Simulate loading progress data
  useEffect(() => {
    const loadProgressData = async () => {
      try {
        setIsLoading(true);

        // In a real implementation, this would call an API to get progress data
        // For now, we'll simulate with mock data
        await new Promise(resolve => setTimeout(resolve, 500)); // Simulate API call

        // Mock progress data
        const mockProgress = {};
        modules.forEach((module, index) => {
          mockProgress[module.id] = {
            moduleId: module.id,
            completed: Math.random() > 0.5, // Random completion for demo
            score: module.id.includes('intro') ? null : Math.floor(Math.random() * 41) + 60, // 60-100 for assessments
            timeSpent: Math.floor(Math.random() * 3600 * 5), // Up to 5 hours
            progressPercentage: Math.floor(Math.random() * 101), // 0-100%
            sectionsCompleted: Math.floor(Math.random() * (module.sectionsCount || 10)),
            totalSections: module.sectionsCount || 10,
            lastAccessed: new Date(Date.now() - Math.floor(Math.random() * 7 * 24 * 60 * 60 * 1000)).toISOString() // Within last week
          };
        });

        setUserProgress(mockProgress);
        setIsLoading(false);
      } catch (err) {
        setError('Failed to load progress data');
        setIsLoading(false);
      }
    };

    if (userId && modules && modules.length > 0) {
      loadProgressData();
    }
  }, [userId, modules]);

  // Handle module selection
  const handleModuleSelect = (module) => {
    setSelectedModule(module);
    if (onModuleSelect) {
      onModuleSelect(module);
    }
  };

  // Format time for display
  const formatTime = (seconds) => {
    if (!seconds) return '0s';

    const hours = Math.floor(seconds / 3600);
    const minutes = Math.floor((seconds % 3600) / 60);
    const remainingSeconds = seconds % 60;

    if (hours > 0) {
      return `${hours}h ${minutes}m ${remainingSeconds}s`;
    } else if (minutes > 0) {
      return `${minutes}m ${remainingSeconds}s`;
    } else {
      return `${remainingSeconds}s`;
    }
  };

  // Get progress bar color based on percentage
  const getProgressColor = (percentage) => {
    if (percentage >= 80) return '#27ae60'; // Green
    if (percentage >= 50) return '#f39c12'; // Orange
    return '#e74c3c'; // Red
  };

  // Get status text
  const getStatusText = (progress) => {
    if (progress.completed) return 'Completed';
    if (progress.progressPercentage >= 80) return 'Almost Done';
    if (progress.progressPercentage >= 50) return 'Halfway';
    if (progress.progressPercentage > 0) return 'In Progress';
    return 'Not Started';
  };

  // Get status icon
  const getStatusIcon = (progress) => {
    if (progress.completed) return '✓';
    if (progress.progressPercentage >= 80) return '→';
    if (progress.progressPercentage > 0) return '○';
    return '○';
  };

  if (isLoading) {
    return (
      <div className="progress-tracker-container">
        <h2>Your Learning Progress</h2>
        <div className="loading-spinner">Loading progress...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="progress-tracker-container">
        <h2>Your Learning Progress</h2>
        <div className="error-message">{error}</div>
      </div>
    );
  }

  return (
    <div className="progress-tracker-container">
      <h2>Your Learning Progress</h2>

      <div className="overall-progress">
        <h3>Overall Course Progress</h3>
        <div className="overall-stats">
          <div className="stat-card">
            <div className="stat-value">
              {modules.length > 0
                ? Math.round(
                    modules.reduce((acc, module) => {
                      const progress = userProgress[module.id];
                      return acc + (progress ? progress.progressPercentage : 0);
                    }, 0) / modules.length
                  )
                : 0}%
            </div>
            <div className="stat-label">Course Complete</div>
          </div>

          <div className="stat-card">
            <div className="stat-value">
              {modules.reduce((acc, module) => {
                const progress = userProgress[module.id];
                return acc + (progress && progress.completed ? 1 : 0);
              }, 0)}
            </div>
            <div className="stat-label">Modules Completed</div>
          </div>

          <div className="stat-card">
            <div className="stat-value">
              {formatTime(
                modules.reduce((acc, module) => {
                  const progress = userProgress[module.id];
                  return acc + (progress ? progress.timeSpent : 0);
                }, 0)
              )}
            </div>
            <div className="stat-label">Time Spent</div>
          </div>
        </div>
      </div>

      <div className="modules-progress">
        <h3>Module Progress</h3>

        {modules.map((module) => {
          const progress = userProgress[module.id] || {
            progressPercentage: 0,
            timeSpent: 0,
            completed: false,
            sectionsCompleted: 0,
            totalSections: module.sectionsCount || 10
          };

          return (
            <div
              key={module.id}
              className={`module-card ${selectedModule?.id === module.id ? 'selected' : ''}`}
              onClick={() => handleModuleSelect(module)}
            >
              <div className="module-header">
                <div className="module-title">
                  <span className="status-icon" style={{ color: getProgressColor(progress.progressPercentage) }}>
                    {getStatusIcon(progress)}
                  </span>
                  {module.title}
                </div>
                <div className="module-status">
                  {getStatusText(progress)}
                </div>
              </div>

              <div className="module-progress-bar">
                <div
                  className="progress-fill"
                  style={{
                    width: `${progress.progressPercentage}%`,
                    backgroundColor: getProgressColor(progress.progressPercentage)
                  }}
                ></div>
                <span className="progress-text">{progress.progressPercentage}%</span>
              </div>

              <div className="module-details">
                <div className="detail-item">
                  <span className="detail-label">Sections:</span>
                  <span className="detail-value">
                    {progress.sectionsCompleted} / {progress.totalSections}
                  </span>
                </div>

                <div className="detail-item">
                  <span className="detail-label">Time:</span>
                  <span className="detail-value">{formatTime(progress.timeSpent)}</span>
                </div>

                {progress.score !== null && (
                  <div className="detail-item">
                    <span className="detail-label">Score:</span>
                    <span className="detail-value score">{progress.score}%</span>
                  </div>
                )}

                <div className="detail-item">
                  <span className="detail-label">Last:</span>
                  <span className="detail-value">
                    {progress.lastAccessed
                      ? new Date(progress.lastAccessed).toLocaleDateString()
                      : 'Never'}
                  </span>
                </div>
              </div>
            </div>
          );
        })}
      </div>

      {selectedModule && (
        <div className="module-details-panel">
          <h3>Detailed Progress: {selectedModule.title}</h3>
          <div className="detailed-progress-info">
            <div className="info-item">
              <strong>Progress:</strong> {userProgress[selectedModule.id]?.progressPercentage || 0}%
            </div>
            <div className="info-item">
              <strong>Time Spent:</strong> {formatTime(userProgress[selectedModule.id]?.timeSpent || 0)}
            </div>
            <div className="info-item">
              <strong>Sections Completed:</strong> {userProgress[selectedModule.id]?.sectionsCompleted || 0} / {userProgress[selectedModule.id]?.totalSections || 10}
            </div>
            {userProgress[selectedModule.id]?.score !== null && (
              <div className="info-item">
                <strong>Latest Assessment Score:</strong> {userProgress[selectedModule.id].score}%
              </div>
            )}
            <div className="info-item">
              <strong>Status:</strong> {userProgress[selectedModule.id]?.completed ? 'Completed' : 'In Progress'}
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default ProgressTracker;