"""
Contract tests for translation endpoint in the Physical AI & Humanoid Robotics textbook.

These tests verify the API contract for the translation functionality between English and Urdu.
"""

import pytest
import requests
from typing import Dict, Any


class TestTranslationContract:
    """
    Contract tests for the translation endpoint.
    Verifies that the translation API follows the expected contract.
    """

    BASE_URL = "http://localhost:3000/api/translate"  # Adjust based on actual API endpoint

    def test_translation_endpoint_exists(self):
        """
        Test that the translation endpoint exists and is accessible.
        """
        response = requests.head(f"{self.BASE_URL}")
        assert response.status_code in [200, 405], f"Endpoint should exist, got status: {response.status_code}"

    def test_translation_with_valid_content(self):
        """
        Test translation with valid content parameters.
        Expected request format:
        {
            "content": "text to translate",
            "source_lang": "en" (optional, default "en"),
            "target_lang": "ur"
        }
        Expected response format:
        {
            "translated_content": "translated text",
            "source_lang": "en",
            "target_lang": "ur",
            "success": true
        }
        """
        payload = {
            "content": "Welcome to Physical AI & Humanoid Robotics textbook",
            "target_lang": "ur"
        }

        response = requests.post(f"{self.BASE_URL}", json=payload)

        # Check response status
        assert response.status_code == 200, f"Expected 200, got {response.status_code}"

        # Parse response
        data = response.json()

        # Validate response structure
        assert "translated_content" in data, "Response must contain 'translated_content' field"
        assert "source_lang" in data, "Response must contain 'source_lang' field"
        assert "target_lang" in data, "Response must contain 'target_lang' field"
        assert "success" in data, "Response must contain 'success' field"

        # Validate field types
        assert isinstance(data["translated_content"], str), "'translated_content' must be a string"
        assert isinstance(data["source_lang"], str), "'source_lang' must be a string"
        assert isinstance(data["target_lang"], str), "'target_lang' must be a string"
        assert isinstance(data["success"], bool), "'success' must be a boolean"

        # Validate content
        assert data["success"] is True, "Translation should be successful"
        assert data["target_lang"] == "ur", "Target language should be Urdu"
        assert len(data["translated_content"]) > 0, "Translated content should not be empty"

    def test_translation_with_different_source_language(self):
        """
        Test translation with explicit source language specification.
        """
        payload = {
            "content": "Robotics is an interdisciplinary field",
            "source_lang": "en",
            "target_lang": "ur"
        }

        response = requests.post(f"{self.BASE_URL}", json=payload)
        assert response.status_code == 200, f"Expected 200, got {response.status_code}"

        data = response.json()
        assert data["success"] is True
        assert data["source_lang"] == "en"
        assert data["target_lang"] == "ur"
        assert len(data["translated_content"]) > 0

    def test_translation_with_invalid_content(self):
        """
        Test translation with invalid content (empty string).
        Should return appropriate error response.
        """
        payload = {
            "content": "",
            "target_lang": "ur"
        }

        response = requests.post(f"{self.BASE_URL}", json=payload)

        # Could be 200 with error in body, or 400 for bad request
        if response.status_code == 200:
            data = response.json()
            assert "success" in data
            assert data["success"] is False
        else:
            assert response.status_code in [400, 422], f"Expected 400 or 422 for invalid content, got {response.status_code}"

    def test_translation_with_invalid_target_language(self):
        """
        Test translation with unsupported target language.
        Should return appropriate error response.
        """
        payload = {
            "content": "Test content",
            "target_lang": "xx"  # Invalid language code
        }

        response = requests.post(f"{self.BASE_URL}", json=payload)

        if response.status_code == 200:
            data = response.json()
            assert "success" in data
            assert data["success"] is False
        else:
            assert response.status_code in [400, 422], f"Expected 400 or 422 for invalid language, got {response.status_code}"

    def test_translation_response_format_consistency(self):
        """
        Test that response format is consistent across different requests.
        """
        test_cases = [
            "Physical AI bridges digital and physical intelligence",
            "Humanoid robots mimic human form and capabilities",
            "ROS 2 provides middleware for robot communication"
        ]

        for content in test_cases:
            payload = {
                "content": content,
                "target_lang": "ur"
            }

            response = requests.post(f"{self.BASE_URL}", json=payload)
            assert response.status_code == 200, f"Request failed for content: {content}"

            data = response.json()

            # Check that all required fields are present
            required_fields = ["translated_content", "source_lang", "target_lang", "success"]
            for field in required_fields:
                assert field in data, f"Required field '{field}' missing in response for content: {content}"

            # Check that field types are consistent
            assert isinstance(data["translated_content"], str)
            assert isinstance(data["source_lang"], str)
            assert isinstance(data["target_lang"], str)
            assert isinstance(data["success"], bool)

    def test_translation_idempotency(self):
        """
        Test that multiple requests with same content return consistent results.
        """
        payload = {
            "content": "Consistent translation test",
            "target_lang": "ur"
        }

        # Make multiple requests
        responses = []
        for _ in range(3):
            response = requests.post(f"{self.BASE_URL}", json=payload)
            assert response.status_code == 200
            responses.append(response.json())

        # Check that all responses have the same translated content
        first_translation = responses[0]["translated_content"]
        for response in responses[1:]:
            assert response["translated_content"] == first_translation, "Translation should be consistent across requests"

    def test_translation_headers(self):
        """
        Test that the endpoint returns appropriate headers.
        """
        payload = {
            "content": "Header test content",
            "target_lang": "ur"
        }

        response = requests.post(f"{self.BASE_URL}", json=payload)
        assert response.status_code == 200

        # Check for common headers
        assert "content-type" in response.headers
        assert "application/json" in response.headers["content-type"], "Response should be JSON"

        # Check for CORS headers if implemented
        # This is optional depending on implementation
        # assert "access-control-allow-origin" in response.headers


if __name__ == "__main__":
    pytest.main([__file__])