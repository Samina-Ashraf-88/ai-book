import React, { useState } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { UserRegistrationData } from '../../models/user';
import { logger } from '../../utils/logger';
import validation from '../../utils/validation';
import './Registration.css';

const Registration: React.FC = () => {
  const { signUp } = useAuth();
  const [formData, setFormData] = useState<UserRegistrationData>({
    email: '',
    password: '',
    softwareBackground: 'None',
    aiExperience: 'None',
    roboticsExperience: 'None',
    programmingLanguages: [],
    hardwareExperience: 'None',
    learningStyle: 'Visual',
  });
  const [selectedLanguages, setSelectedLanguages] = useState<string[]>([]);
  const [newLanguage, setNewLanguage] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData({
      ...formData,
      [name]: value,
    });
  };

  const handleAddLanguage = () => {
    if (newLanguage.trim() && !selectedLanguages.includes(newLanguage.trim())) {
      setSelectedLanguages([...selectedLanguages, newLanguage.trim()]);
      setFormData({
        ...formData,
        programmingLanguages: [...selectedLanguages, newLanguage.trim()],
      });
      setNewLanguage('');
    }
  };

  const handleRemoveLanguage = (language: string) => {
    const updatedLanguages = selectedLanguages.filter(lang => lang !== language);
    setSelectedLanguages(updatedLanguages);
    setFormData({
      ...formData,
      programmingLanguages: updatedLanguages,
    });
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setIsLoading(true);
    setError(null);

    try {
      // Validate the form data
      const validationResults = validation.validateRegistrationForm(
        formData.email,
        formData.password,
        formData.password, // confirmPassword for now, would be separate in real form
        formData.softwareBackground,
        formData.aiExperience,
        formData.roboticsExperience,
        selectedLanguages,
        formData.hardwareExperience,
        formData.learningStyle
      );

      if (!validationResults.isValid) {
        throw new Error(validationResults.errors.join('; '));
      }

      // Add programming languages to form data
      const finalFormData = {
        ...formData,
        programmingLanguages: selectedLanguages,
      };

      // Log registration attempt
      logger.info('Registration attempt', { email: finalFormData.email });

      // Call the actual registration function
      // This would be connected to Better Auth in a real implementation
      await signUp?.();

      // In a real implementation, you would send the form data to your backend
      console.log('Registration data:', finalFormData);

      // Reset form
      setFormData({
        email: '',
        password: '',
        softwareBackground: 'None',
        aiExperience: 'None',
        roboticsExperience: 'None',
        programmingLanguages: [],
        hardwareExperience: 'None',
        learningStyle: 'Visual',
      });
      setSelectedLanguages([]);

      logger.info('Registration successful', { email: finalFormData.email });
    } catch (err: any) {
      logger.error('Registration failed', { error: err.message, email: formData.email });
      setError(err.message || 'Registration failed. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className="registration-container">
      <h2>Register for Physical AI Course</h2>
      {error && <div className="error-message">{error}</div>}

      <form onSubmit={handleSubmit} className="registration-form">
        <div className="form-group">
          <label htmlFor="email">Email:</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleInputChange}
            required
          />
        </div>

        <div className="form-group">
          <label htmlFor="password">Password:</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleInputChange}
            required
            minLength={8}
          />
        </div>

        <div className="form-group">
          <label htmlFor="softwareBackground">Software Development Background:</label>
          <select
            id="softwareBackground"
            name="softwareBackground"
            value={formData.softwareBackground}
            onChange={handleInputChange}
          >
            <option value="None">None</option>
            <option value="Beginner">Beginner</option>
            <option value="Intermediate">Intermediate</option>
            <option value="Advanced">Advanced</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="aiExperience">AI/ML Experience Level:</label>
          <select
            id="aiExperience"
            name="aiExperience"
            value={formData.aiExperience}
            onChange={handleInputChange}
          >
            <option value="None">None</option>
            <option value="Basic ML">Basic ML</option>
            <option value="Deep Learning">Deep Learning</option>
            <option value="Production AI">Production AI</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="roboticsExperience">Robotics Experience:</label>
          <select
            id="roboticsExperience"
            name="roboticsExperience"
            value={formData.roboticsExperience}
            onChange={handleInputChange}
          >
            <option value="None">None</option>
            <option value="Hobby Projects">Hobby Projects</option>
            <option value="Academic">Academic</option>
            <option value="Professional">Professional</option>
          </select>
        </div>

        <div className="form-group">
          <label>Programming Languages Known:</label>
          <div className="language-input">
            <input
              type="text"
              value={newLanguage}
              onChange={(e) => setNewLanguage(e.target.value)}
              placeholder="Add a programming language"
            />
            <button type="button" onClick={handleAddLanguage}>Add</button>
          </div>
          <div className="selected-languages">
            {selectedLanguages.map((lang, index) => (
              <span key={index} className="language-tag">
                {lang}
                <button type="button" onClick={() => handleRemoveLanguage(lang)}>×</button>
              </span>
            ))}
          </div>
        </div>

        <div className="form-group">
          <label htmlFor="hardwareExperience">Hardware Experience:</label>
          <select
            id="hardwareExperience"
            name="hardwareExperience"
            value={formData.hardwareExperience}
            onChange={handleInputChange}
          >
            <option value="None">None</option>
            <option value="Raspberry Pi">Raspberry Pi</option>
            <option value="Arduino">Arduino</option>
            <option value="Jetson">Jetson</option>
            <option value="Custom Boards">Custom Boards</option>
          </select>
        </div>

        <div className="form-group">
          <label htmlFor="learningStyle">Preferred Learning Style:</label>
          <select
            id="learningStyle"
            name="learningStyle"
            value={formData.learningStyle}
            onChange={handleInputChange}
          >
            <option value="Visual">Visual</option>
            <option value="Hands-on">Hands-on</option>
            <option value="Theoretical">Theoretical</option>
            <option value="Mixed">Mixed</option>
          </select>
        </div>

        <button type="submit" disabled={isLoading}>
          {isLoading ? 'Registering...' : 'Register'}
        </button>
      </form>
    </div>
  );
};

export default Registration;