# API Contracts: Physical_Humanoid_AI_Robotics_Course

## Overview

This document defines the API contracts for the conversational AI and robotics integration components of the Physical_Humanoid_AI_Robotics_Course platform. These contracts specify the interfaces between the course platform and the conversational AI systems, as well as other integration points.

## Base API Information

- **Base URL**: `/api/v1`
- **Content-Type**: `application/json`
- **Authentication**: Bearer token required for all endpoints
- **Rate Limiting**: 1000 requests per hour per user

## Authentication

All API endpoints require authentication using a Bearer token:

```
Authorization: Bearer {token}
```

Tokens are obtained through the user authentication system and are valid for 24 hours.

## API Endpoints

### 1. Conversational AI Endpoints

#### POST /conversations/process

Process a user's input through the conversational AI system.

**Request**:
```json
{
  "sessionId": "string",
  "userId": "string",
  "input": {
    "text": "string",
    "modality": "text|speech|gesture|multimodal",
    "timestamp": "ISO 8601 datetime",
    "context": {
      "currentModule": "string",
      "currentLesson": "string",
      "robotState": "object",
      "environment": "object"
    }
  }
}
```

**Response**:
```json
{
  "conversationId": "string",
  "response": {
    "text": "string",
    "intent": "string",
    "entities": {
      "objects": ["string"],
      "locations": ["string"],
      "actions": ["string"]
    },
    "confidence": "float (0-1)",
    "action": {
      "type": "string",
      "parameters": "object"
    }
  },
  "metadata": {
    "processingTime": "integer (ms)",
    "modelUsed": "string",
    "timestamp": "ISO 8601 datetime"
  }
}
```

**Success Response**: 200 OK
**Error Responses**: 
- 400 Bad Request (invalid input)
- 401 Unauthorized (invalid token)
- 429 Too Many Requests (rate limit exceeded)
- 500 Internal Server Error (AI service unavailable)

#### GET /conversations/{conversationId}

Retrieve a specific conversation history.

**Path Parameters**:
- `conversationId`: The ID of the conversation to retrieve

**Response**:
```json
{
  "conversationId": "string",
  "userId": "string",
  "messages": [
    {
      "id": "string",
      "sender": "user|robot",
      "content": "string",
      "timestamp": "ISO 8601 datetime",
      "modality": "text|speech|gesture|multimodal",
      "intent": "string",
      "entities": "object"
    }
  ],
  "createdAt": "ISO 8601 datetime",
  "updatedAt": "ISO 8601 datetime"
}
```

**Success Response**: 200 OK
**Error Responses**: 401, 404, 500

### 2. Progress Tracking Endpoints

#### POST /progress/track

Track user progress in the course.

**Request**:
```json
{
  "userId": "string",
  "moduleId": "string",
  "lessonId": "string",
  "progress": {
    "completed": "boolean",
    "completionPercentage": "float (0-1)",
    "timeSpent": "integer (seconds)",
    "score": "integer (0-100)",
    "attempts": "integer"
  },
  "metadata": {
    "device": "string",
    "platform": "string",
    "timestamp": "ISO 8601 datetime"
  }
}
```

**Response**:
```json
{
  "progressId": "string",
  "userId": "string",
  "moduleId": "string",
  "lessonId": "string",
  "progress": {
    "completed": "boolean",
    "completionPercentage": "float (0-1)",
    "timeSpent": "integer (seconds)",
    "score": "integer (0-100)",
    "attempts": "integer"
  },
  "status": "success|error",
  "message": "string"
}
```

**Success Response**: 200 OK
**Error Responses**: 400, 401, 500

#### GET /progress/user/{userId}

Retrieve progress for a specific user.

**Path Parameters**:
- `userId`: The ID of the user to retrieve progress for

**Query Parameters**:
- `moduleId` (optional): Filter by specific module
- `completed` (optional): Filter by completion status (true/false)

**Response**:
```json
{
  "userId": "string",
  "totalModules": "integer",
  "completedModules": "integer",
  "overallProgress": "float (0-1)",
  "modules": [
    {
      "moduleId": "string",
      "moduleName": "string",
      "completed": "boolean",
      "completionPercentage": "float (0-1)",
      "timeSpent": "integer (seconds)",
      "lastAccessed": "ISO 8601 datetime"
    }
  ]
}
```

**Success Response**: 200 OK
**Error Responses**: 401, 404, 500

### 3. Assessment Endpoints

#### POST /assessments/submit

Submit an assessment response.

**Request**:
```json
{
  "userId": "string",
  "assessmentId": "string",
  "moduleId": "string",
  "responses": [
    {
      "questionId": "string",
      "answer": "string|object",
      "timeTaken": "integer (seconds)"
    }
  ],
  "metadata": {
    "device": "string",
    "platform": "string",
    "timestamp": "ISO 8601 datetime"
  }
}
```

**Response**:
```json
{
  "submissionId": "string",
  "userId": "string",
  "assessmentId": "string",
  "moduleId": "string",
  "score": "integer (0-100)",
  "grade": "string (A-F)",
  "feedback": {
    "overall": "string",
    "perQuestion": [
      {
        "questionId": "string",
        "correct": "boolean",
        "feedback": "string",
        "pointsEarned": "integer",
        "pointsPossible": "integer"
      }
    ]
  },
  "status": "graded|pending",
  "submittedAt": "ISO 8601 datetime"
}
```

**Success Response**: 200 OK
**Error Responses**: 400, 401, 500

#### GET /assessments/{assessmentId}/results

Get results for a specific assessment.

**Path Parameters**:
- `assessmentId`: The ID of the assessment to retrieve results for

**Response**:
```json
{
  "assessmentId": "string",
  "moduleId": "string",
  "averageScore": "float (0-100)",
  "medianScore": "float (0-100)",
  "passRate": "float (0-1)",
  "completionRate": "float (0-1)",
  "timeStats": {
    "averageTime": "integer (seconds)",
    "minTime": "integer (seconds)",
    "maxTime": "integer (seconds)"
  },
  "questionStats": [
    {
      "questionId": "string",
      "averageScore": "float (0-1)",
      "passRate": "float (0-1)",
      "mostCommonErrors": ["string"]
    }
  ]
}
```

**Success Response**: 200 OK
**Error Responses**: 401, 404, 500

### 4. Simulation Integration Endpoints

#### POST /simulations/control

Send control commands to simulation environment.

**Request**:
```json
{
  "sessionId": "string",
  "userId": "string",
  "robotId": "string",
  "command": {
    "type": "move|grasp|navigate|speak|other",
    "parameters": {
      "target": "object",
      "position": {
        "x": "float",
        "y": "float",
        "z": "float"
      },
      "orientation": {
        "qx": "float",
        "qy": "float",
        "qz": "float",
        "qw": "float"
      },
      "velocity": "float",
      "force": "float"
    }
  },
  "context": {
    "taskId": "string",
    "environment": "string"
  }
}
```

**Response**:
```json
{
  "commandId": "string",
  "status": "queued|executing|completed|failed",
  "result": {
    "success": "boolean",
    "message": "string",
    "executionTime": "integer (ms)",
    "robotState": {
      "position": {
        "x": "float",
        "y": "float",
        "z": "float"
      },
      "orientation": {
        "qx": "float",
        "qy": "float",
        "qz": "float",
        "qw": "float"
      },
      "velocity": {
        "x": "float",
        "y": "float",
        "z": "float"
      }
    }
  },
  "timestamp": "ISO 8601 datetime"
}
```

**Success Response**: 200 OK
**Error Responses**: 400, 401, 422 (validation error), 500

### 5. Certification Endpoints

#### POST /certificates/generate

Generate a certificate for course completion.

**Request**:
```json
{
  "userId": "string",
  "courseId": "string",
  "criteriaMet": {
    "modulesCompleted": "integer",
    "assessmentsPassed": "integer",
    "projectsCompleted": "integer",
    "overallScore": "float (0-100)"
  },
  "metadata": {
    "issuingAuthority": "string",
    "issueDate": "ISO 8601 datetime"
  }
}
```

**Response**:
```json
{
  "certificateId": "string",
  "userId": "string",
  "courseId": "string",
  "courseName": "string",
  "userName": "string",
  "issueDate": "ISO 8601 datetime",
  "validUntil": "ISO 8601 datetime (null if permanent)",
  "qrCode": "string (base64 encoded QR code)",
  "verificationUrl": "string",
  "metadata": {
    "issuingAuthority": "string",
    "criteriaMet": "object"
  }
}
```

**Success Response**: 201 Created
**Error Responses**: 400, 401, 403 (not eligible), 500

#### GET /certificates/{certificateId}

Retrieve a specific certificate.

**Path Parameters**:
- `certificateId`: The ID of the certificate to retrieve

**Response**:
```json
{
  "certificateId": "string",
  "userId": "string",
  "courseId": "string",
  "courseName": "string",
  "userName": "string",
  "issueDate": "ISO 8601 datetime",
  "validUntil": "ISO 8601 datetime (null if permanent)",
  "qrCode": "string (base64 encoded QR code)",
  "verificationUrl": "string",
  "metadata": {
    "issuingAuthority": "string",
    "criteriaMet": "object"
  },
  "signature": "string (digital signature)"
}
```

**Success Response**: 200 OK
**Error Responses**: 401, 404, 500

## Error Response Format

All error responses follow this format:

```json
{
  "error": {
    "code": "string",
    "message": "string",
    "details": "object (optional)",
    "timestamp": "ISO 8601 datetime"
  }
}
```

## Common Error Codes

- `INVALID_INPUT`: Request data doesn't meet validation requirements
- `AUTHENTICATION_FAILED`: Invalid or expired authentication token
- `RESOURCE_NOT_FOUND`: Requested resource doesn't exist
- `RATE_LIMIT_EXCEEDED`: Request rate limit has been exceeded
- `SERVICE_UNAVAILABLE`: External service (like AI model) is temporarily unavailable
- `INSUFFICIENT_PRIVILEGES`: User doesn't have permission for requested action
- `VALIDATION_ERROR`: Input validation failed with specific details

## Security Considerations

### Rate Limiting
- Per-user rate limiting to prevent abuse
- Burst allowance for legitimate usage patterns
- Gradual increase in limits for trusted users

### Data Privacy
- All user data is encrypted in transit and at rest
- Personal information is anonymized where possible
- Compliance with GDPR and other privacy regulations

### Authentication
- JWT tokens with short expiration times
- Refresh token mechanism for extended sessions
- Secure token storage and transmission

## Performance Expectations

### Response Times
- Conversational AI: <500ms for 95% of requests
- Progress tracking: <200ms for 95% of requests
- Assessment submission: <300ms for 95% of requests
- Simulation control: <100ms for 95% of requests

### Availability
- 99.9% uptime during peak hours
- Planned maintenance windows announced 48 hours in advance
- Automatic failover to backup systems

## Versioning

API versioning follows semantic versioning principles:
- `/api/v1/` for stable endpoints
- Experimental endpoints in `/api/experimental/`
- Deprecation notices provided 3 months before removal

## Testing Endpoints

### Health Check
GET `/health` - Returns system health status

### Test Endpoint
POST `/test` - Test endpoint for verifying API functionality

## Client Implementation Examples

### JavaScript/TypeScript Client

```javascript
class RoboticsCourseAPI {
  constructor(baseUrl, authToken) {
    this.baseUrl = baseUrl;
    this.authToken = authToken;
  }
  
  async processConversation(input, context = {}) {
    const response = await fetch(`${this.baseUrl}/api/v1/conversations/process`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.authToken}`
      },
      body: JSON.stringify({
        sessionId: this.getSessionId(),
        userId: this.getUserId(),
        input: {
          text: input,
          modality: 'text',
          timestamp: new Date().toISOString(),
          context: context
        }
      })
    });
    
    if (!response.ok) {
      throw new Error(`API request failed: ${response.status}`);
    }
    
    return response.json();
  }
  
  async trackProgress(moduleId, lessonId, progressData) {
    const response = await fetch(`${this.baseUrl}/api/v1/progress/track`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${this.authToken}`
      },
      body: JSON.stringify({
        userId: this.getUserId(),
        moduleId: moduleId,
        lessonId: lessonId,
        progress: progressData,
        metadata: {
          device: navigator.userAgent,
          platform: 'web',
          timestamp: new Date().toISOString()
        }
      })
    });
    
    if (!response.ok) {
      throw new Error(`Progress tracking failed: ${response.status}`);
    }
    
    return response.json();
  }
}
```

### Python Client

```python
import requests
import json
from datetime import datetime

class RoboticsCourseClient:
    def __init__(self, base_url, auth_token):
        self.base_url = base_url
        self.auth_token = auth_token
        self.session = requests.Session()
        self.session.headers.update({
            'Authorization': f'Bearer {auth_token}',
            'Content-Type': 'application/json'
        })
    
    def process_conversation(self, input_text, context=None):
        """Process a conversation with the AI system"""
        payload = {
            'sessionId': self.get_session_id(),
            'userId': self.get_user_id(),
            'input': {
                'text': input_text,
                'modality': 'text',
                'timestamp': datetime.utcnow().isoformat() + 'Z',
                'context': context or {}
            }
        }
        
        response = self.session.post(
            f'{self.base_url}/api/v1/conversations/process',
            json=payload
        )
        
        response.raise_for_status()
        return response.json()
    
    def submit_assessment(self, assessment_id, responses):
        """Submit responses to an assessment"""
        payload = {
            'userId': self.get_user_id(),
            'assessmentId': assessment_id,
            'responses': responses,
            'metadata': {
                'device': 'python-client',
                'platform': 'desktop',
                'timestamp': datetime.utcnow().isoformat() + 'Z'
            }
        }
        
        response = self.session.post(
            f'{self.base_url}/api/v1/assessments/submit',
            json=payload
        )
        
        response.raise_for_status()
        return response.json()
```

## Integration Examples

### ROS 2 Integration

```python
# Example: ROS 2 node that communicates with the API
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from your_msgs.msg import RobotCommand
import requests
import json

class ConversationalAIClient(Node):
    def __init__(self):
        super().__init__('conversational_ai_client')
        
        # Publishers and subscribers
        self.speech_sub = self.create_subscription(
            String,
            'speech_to_text',
            self.speech_callback,
            10
        )
        
        self.ai_response_pub = self.create_publisher(
            String,
            'ai_response',
            10
        )
        
        self.robot_cmd_pub = self.create_publisher(
            RobotCommand,
            'robot_command',
            10
        )
        
        # API configuration
        self.api_base_url = 'http://localhost:3000/api/v1'
        self.auth_token = 'your-token-here'
        
    def speech_callback(self, msg):
        """Process incoming speech from STT system"""
        try:
            # Process through conversational AI API
            response = self.process_conversation(msg.data)
            
            # Publish AI response
            ai_response_msg = String()
            ai_response_msg.data = response['response']['text']
            self.ai_response_pub.publish(ai_response_msg)
            
            # Execute any robot actions
            if 'action' in response['response']:
                self.execute_robot_action(response['response']['action'])
                
        except Exception as e:
            self.get_logger().error(f'Error processing conversation: {e}')
    
    def process_conversation(self, input_text):
        """Call the conversational AI API"""
        headers = {
            'Authorization': f'Bearer {self.auth_token}',
            'Content-Type': 'application/json'
        }
        
        payload = {
            'sessionId': self.get_session_id(),
            'userId': self.get_user_id(),
            'input': {
                'text': input_text,
                'modality': 'speech',
                'timestamp': self.get_timestamp(),
                'context': self.get_robot_context()
            }
        }
        
        response = requests.post(
            f'{self.api_base_url}/conversations/process',
            headers=headers,
            json=payload
        )
        
        response.raise_for_status()
        return response.json()
    
    def execute_robot_action(self, action):
        """Execute robot action based on AI response"""
        cmd_msg = RobotCommand()
        cmd_msg.action_type = action['type']
        cmd_msg.parameters = json.dumps(action['parameters'])
        
        self.robot_cmd_pub.publish(cmd_msg)
```

## Summary

These API contracts define the interfaces for the conversational AI and robotics integration components of the Physical_Humanoid_AI_Robotics_Course platform. They enable:

- Natural language interaction with the course content
- Progress tracking and assessment submission
- Simulation control integration
- Certificate generation and verification
- Secure and scalable communication between components

The contracts follow RESTful principles and include appropriate error handling, authentication, and performance considerations. They provide a solid foundation for building the conversational AI integration components of the course platform.