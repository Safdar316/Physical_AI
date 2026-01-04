---
title: GPT Integration for Robotics
sidebar_label: GPT Integration for Robotics
description: Integrating GPT models with robotic systems for natural language interaction
keywords: [gpt, ai, robotics, natural language, human-robot interaction, conversational ai]
---

## Learning Objectives

- Understand how to integrate GPT models with robotic systems
- Implement natural language processing for robot control
- Design effective human-robot dialogue systems
- Create multimodal interaction patterns
- Implement safety and ethical considerations in conversational AI

## Introduction

Integrating GPT models with robotic systems opens up new possibilities for natural human-robot interaction. This module explores how to leverage large language models to create robots that can understand and respond to natural language commands, engage in conversations, and assist humans in complex tasks.

GPT models can enhance robotics applications by:
- Providing natural language interfaces
- Enabling contextual understanding
- Facilitating human-like interaction patterns
- Supporting complex task planning through dialogue

## Understanding GPT in Robotics Context

### Capabilities of GPT Models

GPT models bring several capabilities to robotics:

1. **Natural Language Understanding**: Interpreting human commands and questions
2. **Contextual Reasoning**: Understanding context and maintaining conversation state
3. **Task Planning**: Breaking down complex requests into executable actions
4. **Knowledge Retrieval**: Accessing vast amounts of world knowledge
5. **Generation**: Creating natural responses and explanations

### Limitations and Considerations

When using GPT in robotics, consider:

- **Latency**: Language models may introduce delays in response
- **Hallucinations**: Models may generate incorrect information
- **Safety**: Ensuring robot actions based on language understanding are safe
- **Real-time Requirements**: Balancing sophisticated processing with real-time needs
- **Context Window**: Managing conversation history within model constraints

## Integration Architecture

### High-Level Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Human User    │───▶│  GPT Model      │───▶│  Robot Actions  │
│                 │    │  (Language      │    │  Interpreter    │
│  Speech/Text    │    │  Processing)    │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                             │
                             ▼
                      ┌─────────────────┐
                      │  Context &      │
                      │  World State    │
                      │  Manager        │
                      └─────────────────┘
```

### Integration Patterns

1. **Direct Command Mapping**: Mapping natural language to specific robot commands
2. **Intent Recognition**: Using language models to identify user intents
3. **Task Decomposition**: Breaking complex requests into sequential actions
4. **Contextual Understanding**: Maintaining conversation context and world state

## Implementation Approaches

### 1. Command-Based Integration

Simple approach mapping language to specific robot commands:

```python
import openai
import json
from typing import Dict, List, Optional

class GPTRobotCommander:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.robot_capabilities = {
            "navigation": ["move_to", "go_to", "navigate_to"],
            "manipulation": ["pick_up", "place", "grasp", "release"],
            "interaction": ["greet", "wave", "speak", "listen"],
            "perception": ["look_at", "find", "detect", "recognize"]
        }
    
    def process_command(self, user_input: str, robot_state: Dict) -> Dict:
        """
        Process natural language command and convert to robot action
        """
        # Define the prompt for GPT
        prompt = f"""
        You are a robot command interpreter. Convert the following user request into a robot command.
        
        Robot capabilities: {json.dumps(self.robot_capabilities)}
        Current robot state: {json.dumps(robot_state)}
        User request: "{user_input}"
        
        Respond with a JSON object containing:
        - "action": the robot action to perform
        - "parameters": parameters for the action
        - "confidence": confidence level (0-1)
        - "explanation": brief explanation of your interpretation
        
        Example response:
        {{
            "action": "navigate_to",
            "parameters": {{"location": "kitchen_table"}},
            "confidence": 0.9,
            "explanation": "User wants robot to go to the kitchen table"
        }}
        """
        
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo",
                messages=[{"role": "user", "content": prompt}],
                max_tokens=200,
                temperature=0.3
            )
            
            # Parse the response
            action_json = json.loads(response.choices[0].message.content)
            return action_json
            
        except Exception as e:
            print(f"Error processing command: {e}")
            return {
                "action": "none",
                "parameters": {},
                "confidence": 0.0,
                "explanation": f"Error processing command: {str(e)}"
            }
```

### 2. Intent Recognition and State Management

More sophisticated approach with intent recognition:

```python
class GPTIntentRecognizer:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.intents = {
            "navigation": {
                "examples": [
                    "Go to the kitchen", 
                    "Move to the living room", 
                    "Navigate to the charging station"
                ]
            },
            "manipulation": {
                "examples": [
                    "Pick up the red cup", 
                    "Place the book on the shelf", 
                    "Give me the pen"
                ]
            },
            "information_request": {
                "examples": [
                    "What time is it?", 
                    "Tell me about the weather", 
                    "How many items are on the table?"
                ]
            },
            "social_interaction": {
                "examples": [
                    "Say hello to John", 
                    "Introduce yourself", 
                    "Tell me a joke"
                ]
            }
        }
    
    def recognize_intent(self, user_input: str, conversation_context: List[Dict]) -> Dict:
        """
        Recognize intent from user input using GPT
        """
        prompt = f"""
        You are an intent recognition system for a robot. Identify the intent of the user's request.
        
        Available intents: {json.dumps(list(self.intents.keys()))}
        Intent examples: {json.dumps(self.intents)}
        Conversation context: {json.dumps(conversation_context[-5:])}  # Last 5 exchanges
        User input: "{user_input}"
        
        Respond with a JSON object containing:
        - "intent": the recognized intent
        - "confidence": confidence level (0-1)
        - "extracted_entities": dictionary of extracted entities
        - "relevant_context": relevant information from conversation history
        """
        
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=250,
            temperature=0.2
        )
        
        return json.loads(response.choices[0].message.content)
    
    def generate_response(self, intent: str, entities: Dict, robot_state: Dict) -> str:
        """
        Generate appropriate response based on intent and context
        """
        prompt = f"""
        You are a helpful robot assistant. Generate an appropriate response based on the intent and context.
        
        Intent: {intent}
        Entities: {json.dumps(entities)}
        Robot state: {json.dumps(robot_state)}
        
        Generate a natural, helpful response. If the robot needs to perform an action, 
        acknowledge it. If asking for clarification, be specific.
        """
        
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=100,
            temperature=0.7
        )
        
        return response.choices[0].message.content
```

### 3. Task Decomposition and Planning

Breaking complex requests into sequences of actions:

```python
class GPTTaskPlanner:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.robot_actions = [
            "move_to(location)",
            "pick_up(object, location)",
            "place(object, location)",
            "grasp(object)",
            "release(object)",
            "detect_object(object_type, location)",
            "navigate_to(location)",
            "wait(duration)",
            "speak(text)",
            "listen_for_command()"
        ]
    
    def decompose_task(self, high_level_request: str, available_actions: List[str]) -> List[Dict]:
        """
        Decompose a high-level request into a sequence of robot actions
        """
        prompt = f"""
        You are a task decomposition system for a robot. Break down the following high-level request 
        into a sequence of specific robot actions from the available action list.
        
        Available actions: {json.dumps(available_actions)}
        High-level request: "{high_level_request}"
        
        Respond with a JSON array of action objects, each containing:
        - "action": the action name
        - "parameters": parameters for the action
        - "reasoning": why this action is needed
        - "expected_outcome": what should happen after this action
        
        Example for "Bring me the red cup from the kitchen":
        [
          {{
            "action": "navigate_to",
            "parameters": {{"location": "kitchen"}},
            "reasoning": "Need to go to kitchen to find the cup",
            "expected_outcome": "Robot is in the kitchen"
          }},
          {{
            "action": "detect_object", 
            "parameters": {{"object_type": "cup", "location": "kitchen"}},
            "reasoning": "Need to locate the red cup",
            "expected_outcome": "Robot has located the red cup"
          }},
          {{
            "action": "pick_up",
            "parameters": {{"object": "red_cup", "location": "kitchen"}},
            "reasoning": "Need to grasp the cup to transport it",
            "expected_outcome": "Robot is holding the red cup"
          }},
          {{
            "action": "navigate_to",
            "parameters": {{"location": "user"}},
            "reasoning": "Need to bring the cup to the user",
            "expected_outcome": "Robot is near the user"
          }},
          {{
            "action": "place", 
            "parameters": {{"object": "red_cup", "location": "user"}},
            "reasoning": "Need to deliver the cup to the user",
            "expected_outcome": "Cup is placed near the user"
          }}
        ]
        """
        
        response = openai.ChatCompletion.create(
            model="gpt-4",  # Using GPT-4 for more complex reasoning
            messages=[{"role": "user", "content": prompt}],
            max_tokens=1000,
            temperature=0.1
        )
        
        return json.loads(response.choices[0].message.content)
```

## Safety and Ethical Considerations

### Safety Measures

When integrating GPT with robotics, implement safety measures:

```python
class SafetyFilter:
    def __init__(self):
        self.prohibited_actions = [
            "harm", "damage", "injure", "attack", "destroy", 
            "unsafe", "dangerous", "hazardous"
        ]
        self.sensitive_contexts = [
            "medical", "emergency", "dangerous", "hazardous", "unsafe"
        ]
    
    def check_action_safety(self, action: Dict, context: Dict) -> Dict:
        """
        Check if an action is safe to execute
        """
        # Check for prohibited words in action description
        action_desc = f"{action.get('action', '')} {json.dumps(action.get('parameters', {}))}"
        action_desc_lower = action_desc.lower()
        
        for prohibited in self.prohibited_actions:
            if prohibited in action_desc_lower:
                return {
                    "safe": False,
                    "reason": f"Action contains prohibited term: {prohibited}",
                    "suggested_alternative": "Please rephrase your request to avoid unsafe language"
                }
        
        # Check for sensitive contexts
        context_desc = json.dumps(context).lower()
        is_sensitive = any(sensitive in context_desc for sensitive in self.sensitive_contexts)
        
        if is_sensitive:
            # Require higher confidence for sensitive contexts
            confidence = action.get('confidence', 0)
            if confidence < 0.8:
                return {
                    "safe": False,
                    "reason": "Low confidence in sensitive context",
                    "suggested_alternative": "Please provide more specific instructions"
                }
        
        return {"safe": True, "reason": "Action appears safe"}
```

### Ethical Guidelines

Implement ethical guidelines for conversational AI:

```python
class EthicalGuidelines:
    def __init__(self):
        self.guidelines = {
            "transparency": "Be clear about robot capabilities and limitations",
            "privacy": "Respect user privacy and data protection",
            "non_harm": "Never perform actions that could harm humans or property",
            "consent": "Obtain appropriate consent before performing actions",
            "fairness": "Treat all users fairly regardless of personal characteristics"
        }
    
    def evaluate_request_ethics(self, user_request: str, robot_action: Dict) -> Dict:
        """
        Evaluate if a request and proposed action align with ethical guidelines
        """
        evaluation = {
            "ethical": True,
            "violations": [],
            "suggestions": []
        }
        
        # Check for potential guideline violations
        request_lower = user_request.lower()
        
        if "secret" in request_lower or "private" in request_lower:
            evaluation["suggestions"].append(
                "Ensure any information gathering respects privacy policies"
            )
        
        action_name = robot_action.get("action", "").lower()
        if "follow" in action_name or "track" in action_name:
            evaluation["suggestions"].append(
                "Verify explicit consent before following or tracking users"
            )
        
        return evaluation
```

## Practical Implementation Example

### Complete Integration Example

```python
import asyncio
import json
from typing import Dict, List, Optional
from dataclasses import dataclass

@dataclass
class RobotState:
    position: Dict[str, float]
    battery_level: float
    held_object: Optional[str]
    last_interaction: str

class ConversationalRobot:
    def __init__(self, gpt_api_key: str):
        self.gpt_commander = GPTRobotCommander(gpt_api_key)
        self.intent_recognizer = GPTIntentRecognizer(gpt_api_key)
        self.task_planner = GPTTaskPlanner(gpt_api_key)
        self.safety_filter = SafetyFilter()
        self.ethics_checker = EthicalGuidelines()
        
        # Conversation context
        self.conversation_history: List[Dict] = []
        self.robot_state = RobotState(
            position={"x": 0.0, "y": 0.0, "theta": 0.0},
            battery_level=1.0,
            held_object=None,
            last_interaction=""
        )
    
    async def process_user_input(self, user_input: str) -> str:
        """
        Process user input and generate robot response
        """
        # Add to conversation history
        self.conversation_history.append({
            "speaker": "user",
            "text": user_input,
            "timestamp": asyncio.get_event_loop().time()
        })
        
        # Recognize intent
        intent_result = self.intent_recognizer.recognize_intent(
            user_input, 
            self.conversation_history
        )
        
        # If it's a simple command, try direct mapping first
        if intent_result["confidence"] > 0.7:
            action = self.gpt_commander.process_command(user_input, self.robot_state.__dict__)
            
            # Check safety
            safety_check = self.safety_filter.check_action_safety(action, self.robot_state.__dict__)
            if not safety_check["safe"]:
                return safety_check["suggested_alternative"]
            
            # Check ethics
            ethics_check = self.ethics_checker.evaluate_request_ethics(user_input, action)
            
            # Execute action
            result = await self.execute_action(action)
            
            # Generate response
            response = self.intent_recognizer.generate_response(
                intent_result["intent"], 
                intent_result["extracted_entities"], 
                self.robot_state.__dict__
            )
            
            # Add to conversation history
            self.conversation_history.append({
                "speaker": "robot",
                "text": response,
                "action_taken": action,
                "timestamp": asyncio.get_event_loop().time()
            })
            
            return response
        else:
            # For complex requests, use task planning
            try:
                action_sequence = self.task_planner.decompose_task(
                    user_input, 
                    self.gpt_commander.robot_capabilities
                )
                
                # Execute sequence safely
                for action in action_sequence:
                    safety_check = self.safety_filter.check_action_safety(action, self.robot_state.__dict__)
                    if not safety_check["safe"]:
                        return f"Cannot execute action: {safety_check['reason']}"
                    
                    await self.execute_action(action)
                
                return f"I've completed the requested task: {user_input}"
                
            except Exception as e:
                return f"I'm not sure how to perform that task. Could you please be more specific?"
    
    async def execute_action(self, action: Dict) -> Dict:
        """
        Execute robot action (simulated)
        """
        action_name = action.get("action")
        parameters = action.get("parameters", {})
        
        # In a real implementation, this would interface with the robot
        # For now, we'll simulate the action and update robot state
        
        if action_name == "navigate_to":
            location = parameters.get("location", "unknown")
            self.robot_state.position = {"x": 1.0, "y": 1.0, "theta": 0.0}  # Simulated movement
            return {"status": "completed", "result": f"Navigated to {location}"}
        
        elif action_name == "pick_up":
            obj = parameters.get("object", "unknown")
            self.robot_state.held_object = obj
            return {"status": "completed", "result": f"Picked up {obj}"}
        
        elif action_name == "speak":
            text = parameters.get("text", "")
            return {"status": "completed", "result": f"Spoke: {text}"}
        
        else:
            return {"status": "unknown_action", "result": f"Unknown action: {action_name}"}
    
    def get_conversation_context(self) -> List[Dict]:
        """
        Get recent conversation context
        """
        return self.conversation_history[-10:]  # Last 10 exchanges
```

## Multimodal Integration

### Combining Vision and Language

Integrating visual information with language understanding:

```python
class MultimodalGPTIntegration:
    def __init__(self, gpt_api_key: str):
        openai.api_key = gpt_api_key
    
    def describe_scene(self, image_data: str, user_query: str = "") -> str:
        """
        Use GPT-4 Vision to describe a scene and answer questions about it
        """
        # In a real implementation, image_data would be the actual image
        # For this example, we'll use the OpenAI API with vision capabilities
        prompt = f"Describe this image and answer the following question if provided: {user_query}"
        
        response = openai.ChatCompletion.create(
            model="gpt-4-vision-preview",
            messages=[
                {
                    "role": "user",
                    "content": [
                        {"type": "text", "text": prompt},
                        {
                            "type": "image_url",
                            "image_url": {
                                "url": f"data:image/jpeg;base64,{image_data}"
                            }
                        }
                    ]
                }
            ],
            max_tokens=300
        )
        
        return response.choices[0].message.content
    
    def generate_vision_guided_response(self, image_description: str, user_input: str, robot_state: Dict) -> str:
        """
        Generate response based on visual information and user input
        """
        prompt = f"""
        You are a robot assistant with visual perception capabilities.
        
        Visual scene description: {image_description}
        User input: {user_input}
        Current robot state: {json.dumps(robot_state)}
        
        Generate a response that takes into account what the robot can see
        and the user's request. If appropriate, suggest actions based on
        the visual information.
        """
        
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            max_tokens=150,
            temperature=0.7
        )
        
        return response.choices[0].message.content
```

## Performance Optimization

### Caching and Efficiency

```python
import functools
import time
from typing import Callable

class GPTCache:
    def __init__(self, ttl: int = 300):  # 5 minute TTL
        self.cache = {}
        self.ttl = ttl
    
    def get(self, key: str):
        if key in self.cache:
            value, timestamp = self.cache[key]
            if time.time() - timestamp < self.ttl:
                return value
            else:
                del self.cache[key]
        return None
    
    def set(self, key: str, value):
        self.cache[key] = (value, time.time())

class OptimizedGPTIntegration:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.cache = GPTCache()
    
    def cached_gpt_call(self, prompt: str, model: str = "gpt-3.5-turbo", max_tokens: int = 150):
        """
        Make a GPT call with caching for repeated prompts
        """
        # Create a cache key based on prompt and parameters
        cache_key = f"{prompt[:50]}_{model}_{max_tokens}"
        
        # Check cache first
        cached_result = self.cache.get(cache_key)
        if cached_result:
            return cached_result
        
        # Make the API call
        response = openai.ChatCompletion.create(
            model=model,
            messages=[{"role": "user", "content": prompt}],
            max_tokens=max_tokens,
            temperature=0.7
        )
        
        result = response.choices[0].message.content
        
        # Cache the result
        self.cache.set(cache_key, result)
        
        return result
```

## Error Handling and Fallbacks

### Robust Error Handling

```python
import logging
from tenacity import retry, stop_after_attempt, wait_exponential

class RobustGPTIntegration:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.logger = logging.getLogger(__name__)
    
    @retry(stop=stop_after_attempt(3), wait=wait_exponential(multiplier=1, min=4, max=10))
    def robust_gpt_call(self, prompt: str, model: str = "gpt-3.5-turbo"):
        """
        Make GPT calls with retry logic and error handling
        """
        try:
            response = openai.ChatCompletion.create(
                model=model,
                messages=[{"role": "user", "content": prompt}],
                max_tokens=200,
                temperature=0.7
            )
            return response.choices[0].message.content
        except openai.error.RateLimitError:
            self.logger.warning("Rate limit exceeded, will retry...")
            raise
        except openai.error.APIError as e:
            self.logger.error(f"API error: {e}")
            raise
        except Exception as e:
            self.logger.error(f"Unexpected error: {e}")
            return "Sorry, I'm having trouble processing your request right now. Could you please try again?"
    
    def generate_fallback_response(self, user_input: str, error_context: str = ""):
        """
        Generate a fallback response when GPT integration fails
        """
        # Simple rule-based fallback
        user_lower = user_input.lower()
        
        if any(word in user_lower for word in ["hello", "hi", "hey"]):
            return "Hello! I'm your robot assistant. How can I help you today?"
        elif any(word in user_lower for word in ["help", "assist"]):
            return "I can help with navigation, object manipulation, information, and more. What would you like me to do?"
        elif any(word in user_lower for word in ["name", "who are you"]):
            return "I'm a humanoid robot assistant powered by AI. You can ask me to perform tasks or answer questions."
        else:
            return "I'm having some technical difficulties right now. Could you please rephrase your request?"
```

## Testing and Validation

### Unit Tests for GPT Integration

```python
import unittest.mock as mock
import pytest

class TestGPTIntegration:
    def setup_method(self):
        self.gpt_robot = ConversationalRobot("fake_api_key")
    
    @mock.patch('openai.ChatCompletion.create')
    def test_process_simple_command(self, mock_openai):
        # Mock the GPT response
        mock_response = mock.Mock()
        mock_response.choices = [mock.Mock()]
        mock_response.choices[0].message.content = json.dumps({
            "action": "navigate_to",
            "parameters": {"location": "kitchen"},
            "confidence": 0.9,
            "explanation": "User wants robot to go to kitchen"
        })
        mock_openai.return_value = mock_response
        
        result = asyncio.run(self.gpt_robot.process_user_input("Go to the kitchen"))
        
        assert "navigate_to" in result
        assert self.gpt_robot.robot_state.position["x"] == 1.0  # Updated by execute_action
    
    def test_safety_filter_blocks_harmful_commands(self):
        safety_filter = SafetyFilter()
        
        result = safety_filter.check_action_safety(
            {"action": "attack", "parameters": {"target": "person"}},
            {}
        )
        
        assert result["safe"] is False
        assert "harm" in result["reason"] or "attack" in result["reason"]
    
    @mock.patch('openai.ChatCompletion.create')
    def test_intent_recognition(self, mock_openai):
        mock_response = mock.Mock()
        mock_response.choices = [mock.Mock()]
        mock_response.choices[0].message.content = json.dumps({
            "intent": "navigation",
            "confidence": 0.95,
            "extracted_entities": {"location": "kitchen"},
            "relevant_context": "user wants robot to move"
        })
        mock_openai.return_value = mock_response
        
        recognizer = GPTIntentRecognizer("fake_api_key")
        result = recognizer.recognize_intent("Go to the kitchen", [])
        
        assert result["intent"] == "navigation"
        assert result["confidence"] > 0.9
```

## Best Practices

### 1. Context Management

```python
class ContextManager:
    def __init__(self, max_context_length: int = 50):
        self.context_history = []
        self.max_length = max_context_length
    
    def add_context(self, entry: Dict):
        """
        Add a new context entry, maintaining history size
        """
        self.context_history.append(entry)
        if len(self.context_history) > self.max_length:
            self.context_history = self.context_history[-self.max_length:]
    
    def get_recent_context(self, num_entries: int = 10) -> List[Dict]:
        """
        Get recent context entries
        """
        return self.context_history[-num_entries:]
    
    def summarize_context(self) -> str:
        """
        Create a summary of the conversation context
        """
        # Implement context summarization logic
        # This could use GPT to create a summary of the conversation
        pass
```

### 2. Response Verification

```python
class ResponseVerifier:
    def __init__(self):
        self.verification_rules = [
            lambda resp: len(resp) > 0,  # Not empty
            lambda resp: len(resp) < 500,  # Not too long
            lambda resp: not any(bad_word in resp.lower() for bad_word in ["error", "unknown", "unable"]),
        ]
    
    def verify_response(self, response: str) -> bool:
        """
        Verify that a GPT response is appropriate
        """
        for rule in self.verification_rules:
            if not rule(response):
                return False
        return True
    
    def sanitize_response(self, response: str) -> str:
        """
        Sanitize response to remove inappropriate content
        """
        # Implement sanitization logic
        sanitized = response.strip()
        return sanitized
```

## Integration with ROS 2

### ROS 2 Bridge for GPT Integration

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from your_messages.msg import RobotCommand

class GPTROS2Bridge(Node):
    def __init__(self):
        super().__init__('gpt_ros2_bridge')
        
        # Publishers
        self.command_publisher = self.create_publisher(RobotCommand, 'robot_command', 10)
        self.speech_publisher = self.create_publisher(String, 'tts_input', 10)
        
        # Subscribers
        self.voice_subscriber = self.create_subscription(
            String,
            'stt_output',
            self.voice_callback,
            10
        )
        
        # Initialize GPT integration
        self.conversational_robot = ConversationalRobot("your_api_key_here")
        
    def voice_callback(self, msg):
        """
        Process voice input from speech-to-text system
        """
        user_input = msg.data
        
        # Process with GPT integration
        response = asyncio.run(self.conversational_robot.process_user_input(user_input))
        
        # Publish speech response
        speech_msg = String()
        speech_msg.data = response
        self.speech_publisher.publish(speech_msg)
        
        self.get_logger().info(f'Processed: {user_input} -> {response}')

def main(args=None):
    rclpy.init(args=args)
    
    gpt_bridge = GPTROS2Bridge()
    
    try:
        rclpy.spin(gpt_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        gpt_bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

GPT integration in robotics provides powerful natural language capabilities that can significantly enhance human-robot interaction. By combining language understanding with robotic action capabilities, we can create more intuitive and accessible robotic systems.

Key considerations for successful GPT integration in robotics:
- Proper safety and ethical safeguards
- Efficient context management
- Robust error handling and fallbacks
- Appropriate response verification
- Performance optimization for real-time requirements
- Multimodal integration for richer interaction

The integration patterns covered in this module provide a foundation for implementing conversational AI in robotic systems while maintaining safety and reliability.

## Further Reading

- "Language Models for Robotics" - Recent research on LLMs in robotics
- "Safe Human-Robot Interaction" - Guidelines for safe interaction design
- "Multimodal Learning for Robotics" - Combining vision, language, and action
- OpenAI API Documentation for best practices

## Assessment

Complete the practical exercise to implement a simple GPT-robot integration for a basic navigation task.