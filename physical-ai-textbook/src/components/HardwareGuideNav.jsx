import React, { useState } from 'react';
import './HardwareGuideNav.css';

/**
 * Hardware Guide Navigation Component
 * Provides navigation for hardware setup guides
 */
const HardwareGuideNav = ({ guides = [], currentGuide = null }) => {
  const [expandedCategory, setExpandedCategory] = useState(null);
  const [searchTerm, setSearchTerm] = useState('');

  // Predefined guide categories and structure
  const guideStructure = [
    {
      id: 'workstation',
      title: 'Workstation Requirements',
      description: 'Setting up a workstation for Physical AI development',
      difficulty: 'Beginner to Advanced',
      cost: '$500 - $3000+',
      time: '4-12 hours',
      path: '/docs/hardware-guide/workstation'
    },
    {
      id: 'edge-kit',
      title: 'Edge Kit Requirements',
      description: 'Setting up edge computing for robotics applications',
      difficulty: 'Intermediate to Advanced',
      cost: '$200 - $1500+',
      time: '2-10 hours',
      path: '/docs/hardware-guide/edge-kit'
    },
    {
      id: 'robot-options',
      title: 'Robot Lab Options',
      description: 'Options for setting up a robot laboratory',
      difficulty: 'Beginner to Advanced',
      cost: '$1000 - $20000+',
      time: '2-16 weeks',
      path: '/docs/hardware-guide/robot-options'
    },
    {
      id: 'cloud-vs-onpremise',
      title: 'Cloud vs On-Premise',
      description: 'Comparing cloud and on-premise lab options',
      difficulty: 'Intermediate',
      cost: 'Variable',
      time: 'Planning: 1-4 weeks',
      path: '/docs/hardware-guide/cloud-vs-onpremise'
    }
  ];

  // Filter guides based on search term
  const filteredGuides = guideStructure.filter(guide =>
    guide.title.toLowerCase().includes(searchTerm.toLowerCase()) ||
    guide.description.toLowerCase().includes(searchTerm.toLowerCase()) ||
    guide.difficulty.toLowerCase().includes(searchTerm.toLowerCase()) ||
    guide.cost.toLowerCase().includes(searchTerm.toLowerCase())
  );

  // Get category for a guide
  const getGuideCategory = (guideId) => {
    if (guideId.includes('workstation')) return 'Development Setup';
    if (guideId.includes('edge')) return 'Edge Computing';
    if (guideId.includes('robot')) return 'Robot Platforms';
    if (guideId.includes('cloud')) return 'Infrastructure Options';
    return 'General';
  };

  // Group guides by category
  const groupedGuides = filteredGuides.reduce((acc, guide) => {
    const category = getGuideCategory(guide.id);
    if (!acc[category]) {
      acc[category] = [];
    }
    acc[category].push(guide);
    return acc;
  }, {});

  // Handle guide selection
  const handleGuideSelect = (guide) => {
    // In a real implementation, this would navigate to the guide
    if (typeof window !== 'undefined') {
      window.location.href = guide.path;
    }
  };

  // Toggle category expansion
  const toggleCategory = (category) => {
    setExpandedCategory(expandedCategory === category ? null : category);
  };

  return (
    <div className="hardware-guide-nav">
      <div className="nav-header">
        <h2>Hardware Setup Guides</h2>
        <p>Find the right hardware setup for your Physical AI & Humanoid Robotics projects</p>
      </div>

      <div className="search-section">
        <input
          type="text"
          placeholder="Search hardware guides..."
          value={searchTerm}
          onChange={(e) => setSearchTerm(e.target.value)}
          className="search-input"
        />
      </div>

      <div className="guide-categories">
        {Object.entries(groupedGuides).map(([category, guides]) => (
          <div key={category} className="guide-category">
            <div
              className="category-header"
              onClick={() => toggleCategory(category)}
            >
              <h3>{category}</h3>
              <span className={`expand-icon ${expandedCategory === category ? 'expanded' : ''}`}>
                ▼
              </span>
            </div>

            {expandedCategory === category && (
              <div className="category-guides">
                {guides.map((guide) => (
                  <div
                    key={guide.id}
                    className={`guide-item ${currentGuide === guide.id ? 'active' : ''}`}
                    onClick={() => handleGuideSelect(guide)}
                  >
                    <div className="guide-content">
                      <h4>{guide.title}</h4>
                      <p className="guide-description">{guide.description}</p>
                      <div className="guide-meta">
                        <span className="difficulty">Difficulty: {guide.difficulty}</span>
                        <span className="cost">Cost: {guide.cost}</span>
                        <span className="time">Time: {guide.time}</span>
                      </div>
                    </div>
                    <div className="guide-actions">
                      <button className="view-guide-btn">View Guide</button>
                    </div>
                  </div>
                ))}
              </div>
            )}
          </div>
        ))}
      </div>

      <div className="recommendation-section">
        <h3>Need Help Choosing?</h3>
        <p>Take our hardware assessment to get personalized recommendations based on your needs, budget, and experience level.</p>
        <button className="assessment-btn">Hardware Assessment</button>
      </div>

      <div className="quick-links">
        <h3>Quick Links</h3>
        <ul>
          <li><a href="/docs/hardware-guide/workstation">Workstation Setup</a></li>
          <li><a href="/docs/hardware-guide/edge-kit">Edge Kit Setup</a></li>
          <li><a href="/docs/hardware-guide/robot-options">Robot Lab Options</a></li>
          <li><a href="/docs/hardware-guide/cloud-vs-onpremise">Cloud vs On-Premise</a></li>
        </ul>
      </div>
    </div>
  );
};

export default HardwareGuideNav;