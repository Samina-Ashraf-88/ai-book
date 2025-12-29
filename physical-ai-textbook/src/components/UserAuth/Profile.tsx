import React, { useState, useEffect } from 'react';
import { useAuth } from '../../contexts/AuthContext';
import { UserProfileUpdate } from '../../models/user';
import { logger } from '../../utils/logger';

const Profile: React.FC = () => {
  const { user, isLoggedIn } = useAuth();
  const [profileData, setProfileData] = useState<any>(null);
  const [isEditing, setIsEditing] = useState(false);
  const [formData, setFormData] = useState<UserProfileUpdate>({
    softwareBackground: undefined,
    aiExperience: undefined,
    roboticsExperience: undefined,
    programmingLanguages: undefined,
    hardwareExperience: undefined,
    learningStyle: undefined,
  });
  const [selectedLanguages, setSelectedLanguages] = useState<string[]>([]);
  const [newLanguage, setNewLanguage] = useState('');
  const [error, setError] = useState<string | null>(null);
  const [success, setSuccess] = useState<string | null>(null);

  useEffect(() => {
    if (user) {
      setProfileData(user);
      setFormData({
        softwareBackground: user.softwareBackground,
        aiExperience: user.aiExperience,
        roboticsExperience: user.roboticsExperience,
        programmingLanguages: user.programmingLanguages,
        hardwareExperience: user.hardwareExperience,
        learningStyle: user.learningStyle,
      });
      setSelectedLanguages(user.programmingLanguages || []);
    }
  }, [user]);

  const handleInputChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData({
      ...formData,
      [name]: value,
    });
  };

  const handleAddLanguage = () => {
    if (newLanguage.trim() && !selectedLanguages.includes(newLanguage.trim())) {
      const updatedLanguages = [...selectedLanguages, newLanguage.trim()];
      setSelectedLanguages(updatedLanguages);
      setFormData({
        ...formData,
        programmingLanguages: updatedLanguages,
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

  const handleSave = async () => {
    try {
      setError(null);
      setSuccess(null);

      // Log profile update attempt
      logger.info('Profile update attempt', { userId: user?.id });

      // In a real implementation, you would send the form data to your backend
      console.log('Profile update data:', {
        ...formData,
        programmingLanguages: selectedLanguages,
      });

      // Simulate API call
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Update local state
      setProfileData({
        ...profileData,
        ...formData,
        programmingLanguages: selectedLanguages,
      });

      setSuccess('Profile updated successfully!');
      setIsEditing(false);

      logger.info('Profile updated successfully', { userId: user?.id });
    } catch (err: any) {
      logger.error('Profile update failed', { error: err.message, userId: user?.id });
      setError(err.message || 'Profile update failed. Please try again.');
    }
  };

  if (!isLoggedIn || !profileData) {
    return <div>Please log in to view your profile.</div>;
  }

  return (
    <div className="profile-container">
      <h2>User Profile</h2>

      {error && <div className="error-message">{error}</div>}
      {success && <div className="success-message">{success}</div>}

      <div className="profile-info">
        <h3>Basic Information</h3>
        <p><strong>Email:</strong> {profileData.email}</p>
        <p><strong>Member Since:</strong> {profileData.createdAt ? new Date(profileData.createdAt).toLocaleDateString() : 'N/A'}</p>
      </div>

      <div className="profile-background">
        <h3>Background Information</h3>

        {isEditing ? (
          <div className="edit-form">
            <div className="form-group">
              <label htmlFor="softwareBackground">Software Development Background:</label>
              <select
                id="softwareBackground"
                name="softwareBackground"
                value={formData.softwareBackground || profileData.softwareBackground}
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
                value={formData.aiExperience || profileData.aiExperience}
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
                value={formData.roboticsExperience || profileData.roboticsExperience}
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
                value={formData.hardwareExperience || profileData.hardwareExperience}
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
                value={formData.learningStyle || profileData.learningStyle}
                onChange={handleInputChange}
              >
                <option value="Visual">Visual</option>
                <option value="Hands-on">Hands-on</option>
                <option value="Theoretical">Theoretical</option>
                <option value="Mixed">Mixed</option>
              </select>
            </div>

            <div className="button-group">
              <button onClick={handleSave}>Save Changes</button>
              <button onClick={() => setIsEditing(false)}>Cancel</button>
            </div>
          </div>
        ) : (
          <div className="view-mode">
            <p><strong>Software Background:</strong> {profileData.softwareBackground}</p>
            <p><strong>AI/ML Experience:</strong> {profileData.aiExperience}</p>
            <p><strong>Robotics Experience:</strong> {profileData.roboticsExperience}</p>
            <p><strong>Programming Languages:</strong> {profileData.programmingLanguages?.join(', ') || 'None'}</p>
            <p><strong>Hardware Experience:</strong> {profileData.hardwareExperience}</p>
            <p><strong>Learning Style:</strong> {profileData.learningStyle}</p>

            <button onClick={() => setIsEditing(true)}>Edit Profile</button>
          </div>
        )}
      </div>
    </div>
  );
};

export default Profile;