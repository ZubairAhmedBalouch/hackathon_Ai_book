# Data Model: Physical AI & Humanoid Robotics Textbook

## Entities

### Chapter
- **Description**: A self-contained unit of learning content.
- **Fields**:
    - `title`: String, unique identifier and display title.
    - `slug`: String, URL-friendly identifier derived from title.
    - `overview`: String, a brief summary of the chapter's content.
    - `concepts`: List of Strings, key concepts covered in the chapter.
    - `examples`: List of Strings, practical examples or mini-scenarios.
    - `content`: Markdown text, the full body of the chapter in Docusaurus Markdown format.
- **Relationships**: Belongs to a Module.
- **Validation Rules**:
    - `title` and `content` are mandatory.
    - `content` must be valid Docusaurus Markdown.

### Module
- **Description**: A collection of related chapters, representing a thematic section of the textbook.
- **Fields**:
    - `name`: String, unique name of the module (e.g., "Module 1: ROS2").
    - `chapters`: List of Chapter entities, ordered sequentially.
- **Relationships**: Contains Chapters.
- **Validation Rules**:
    - `name` is mandatory.
    - `chapters` list cannot be empty.

### Concept
- **Description**: A fundamental idea or principle explained within the textbook content.
- **Fields**:
    - `name`: String, the name of the concept.
    - `description`: String, a brief explanation of the concept.
    - `chapter_references`: List of Strings, indicating chapters where this concept is introduced or elaborated.
- **Relationships**: Associated with Chapters.
- **Validation Rules**:
    - `name` and `description` are mandatory.
