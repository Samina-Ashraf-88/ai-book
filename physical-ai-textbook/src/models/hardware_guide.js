/**
 * Hardware Guide Model
 * Represents a hardware guide for the Physical AI & Humanoid Robotics textbook
 */

class HardwareGuide {
  constructor({
    id,
    title,
    category,
    content,
    targetAudience,
    estimatedCost,
    difficultyLevel,
    requiredComponents = [],
    toolsNeeded = [],
    safetyConsiderations = [],
    assemblySteps = [],
    troubleshootingTips = [],
    createdAt,
    updatedAt
  }) {
    this.id = id;
    this.title = title;
    this.category = category; // e.g., "Workstation", "Edge Kit", "Robot Options", "Cloud vs On-Premise"
    this.content = content; // Main content of the guide
    this.targetAudience = targetAudience; // e.g., "Budget", "Standard", "Premium"
    this.estimatedCost = estimatedCost; // e.g., "$500-1000"
    this.difficultyLevel = difficultyLevel; // e.g., "Beginner", "Intermediate", "Advanced"
    this.requiredComponents = requiredComponents; // List of required hardware components
    this.toolsNeeded = toolsNeeded; // List of tools needed for assembly
    this.safetyConsiderations = safetyConsiderations; // Safety warnings and considerations
    this.assemblySteps = assemblySteps; // Step-by-step assembly instructions
    this.troubleshootingTips = troubleshootingTips; // Common issues and solutions
    this.createdAt = createdAt || new Date().toISOString();
    this.updatedAt = updatedAt || new Date().toISOString();
  }

  /**
   * Validate the hardware guide structure
   */
  validate() {
    const errors = [];

    if (!this.id) errors.push('ID is required');
    if (!this.title) errors.push('Title is required');
    if (!this.category) errors.push('Category is required');
    if (!this.content) errors.push('Content is required');
    if (!this.targetAudience) errors.push('Target audience is required');
    if (!this.difficultyLevel) errors.push('Difficulty level is required');

    // Validate difficulty level
    const validDifficultyLevels = ['Beginner', 'Intermediate', 'Advanced'];
    if (!validDifficultyLevels.includes(this.difficultyLevel)) {
      errors.push(`Difficulty level must be one of: ${validDifficultyLevels.join(', ')}`);
    }

    // Validate target audience
    const validTargetAudiences = ['Budget', 'Standard', 'Premium'];
    if (!validTargetAudiences.includes(this.targetAudience)) {
      errors.push(`Target audience must be one of: ${validTargetAudiences.join(', ')}`);
    }

    // Validate required components if provided
    if (this.requiredComponents && !Array.isArray(this.requiredComponents)) {
      errors.push('Required components must be an array');
    }

    // Validate tools needed if provided
    if (this.toolsNeeded && !Array.isArray(this.toolsNeeded)) {
      errors.push('Tools needed must be an array');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }

  /**
   * Get a summary of the hardware guide
   */
  getSummary() {
    return {
      id: this.id,
      title: this.title,
      category: this.category,
      targetAudience: this.targetAudience,
      estimatedCost: this.estimatedCost,
      difficultyLevel: this.difficultyLevel,
      requiredComponentsCount: this.requiredComponents ? this.requiredComponents.length : 0,
      toolsNeededCount: this.toolsNeeded ? this.toolsNeeded.length : 0
    };
  }

  /**
   * Check if this guide matches the user's requirements
   * @param {Object} userRequirements - User's requirements for hardware
   * @returns {boolean} - True if the guide matches the requirements
   */
  matchesRequirements(userRequirements) {
    // Check if target audience matches
    if (userRequirements.budgetLevel && this.targetAudience !== userRequirements.budgetLevel) {
      return false;
    }

    // Check if difficulty level is appropriate
    if (userRequirements.experienceLevel) {
      const experienceMatch = this.isExperienceLevelAppropriate(userRequirements.experienceLevel);
      if (!experienceMatch) return false;
    }

    // Check if category matches
    if (userRequirements.category && this.category !== userRequirements.category) {
      return false;
    }

    return true;
  }

  /**
   * Check if the difficulty level is appropriate for the user's experience
   * @param {string} userExperience - User's experience level
   * @returns {boolean} - True if appropriate
   */
  isExperienceLevelAppropriate(userExperience) {
    const difficultyOrder = {
      'Beginner': 1,
      'Intermediate': 2,
      'Advanced': 3
    };

    const userLevel = difficultyOrder[userExperience] || 0;
    const guideLevel = difficultyOrder[this.difficultyLevel] || 0;

    // User can handle guides at their level or below
    return guideLevel <= userLevel;
  }

  /**
   * Estimate the time required for assembly based on difficulty
   * @returns {number} - Estimated time in hours
   */
  estimateAssemblyTime() {
    switch (this.difficultyLevel) {
      case 'Beginner':
        return 4; // 4 hours for beginner
      case 'Intermediate':
        return 8; // 8 hours for intermediate
      case 'Advanced':
        return 16; // 16 hours for advanced
      default:
        return 8; // Default to 8 hours
    }
  }

  /**
   * Get a formatted list of required components with links if available
   */
  getFormattedComponents() {
    if (!this.requiredComponents || !Array.isArray(this.requiredComponents)) {
      return [];
    }

    return this.requiredComponents.map(component => {
      if (typeof component === 'string') {
        return {
          name: component,
          link: null,
          estimatedCost: null
        };
      } else if (typeof component === 'object') {
        return {
          name: component.name || component.title || '',
          link: component.link || component.url || null,
          estimatedCost: component.cost || component.estimatedCost || null
        };
      }
      return {
        name: '',
        link: null,
        estimatedCost: null
      };
    });
  }

  /**
   * Add a safety consideration
   * @param {string} safetyTip - Safety tip to add
   */
  addSafetyConsideration(safetyTip) {
    if (!this.safetyConsiderations.includes(safetyTip)) {
      this.safetyConsiderations.push(safetyTip);
      this.updatedAt = new Date().toISOString();
    }
  }

  /**
   * Add a troubleshooting tip
   * @param {string} issue - Issue description
   * @param {string} solution - Solution description
   */
  addTroubleshootingTip(issue, solution) {
    const tip = {
      issue,
      solution,
      addedAt: new Date().toISOString()
    };

    this.troubleshootingTips.push(tip);
    this.updatedAt = new Date().toISOString();
  }

  /**
   * Update the guide content
   * @param {Object} updates - Object with properties to update
   */
  update(updates) {
    const updatableFields = [
      'title', 'category', 'content', 'targetAudience',
      'estimatedCost', 'difficultyLevel', 'requiredComponents',
      'toolsNeeded', 'safetyConsiderations', 'assemblySteps',
      'troubleshootingTips'
    ];

    for (const [key, value] of Object.entries(updates)) {
      if (updatableFields.includes(key)) {
        this[key] = value;
      }
    }

    this.updatedAt = new Date().toISOString();
  }

  /**
   * Generate a hardware checklist based on required components
   * @returns {Array} - Array of checklist items
   */
  generateChecklist() {
    const checklist = [];

    if (this.requiredComponents && Array.isArray(this.requiredComponents)) {
      this.requiredComponents.forEach(component => {
        const componentName = typeof component === 'string' ? component : (component.name || component.title);
        checklist.push({
          id: this.generateId(componentName),
          name: componentName,
          checked: false,
          category: 'components'
        });
      });
    }

    if (this.toolsNeeded && Array.isArray(this.toolsNeeded)) {
      this.toolsNeeded.forEach(tool => {
        const toolName = typeof tool === 'string' ? tool : tool;
        checklist.push({
          id: this.generateId(toolName),
          name: toolName,
          checked: false,
          category: 'tools'
        });
      });
    }

    return checklist;
  }

  /**
   * Generate a unique ID for internal items
   * @param {string} prefix - Prefix for the ID
   * @returns {string} - Generated ID
   */
  generateId(prefix = '') {
    const timestamp = Date.now().toString(36);
    const randomPart = Math.random().toString(36).substr(2, 5);
    return prefix ? `${prefix}-${timestamp}-${randomPart}` : `${timestamp}-${randomPart}`;
  }
}

module.exports = HardwareGuide;