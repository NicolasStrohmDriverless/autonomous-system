# Testing

## Pyramid Scheme
We are following a pyramid shaped testing scheme. It has three main layers: Unit Tests, Integration Tests, and End-to-End (E2E) Tests. Each layer serves a different purpose and is used to validate different aspects of the application. The pyramid shape suggests that there should be more tests at the lower levels (Unit Tests) and fewer tests at the higher levels (E2E Tests). 

![Testing Pyramid](assets/testing_pyramid.jpg)

### Unit Tests

- **Purpose:** <br> Unit tests are designed to test individual components or functions of the code in isolation. They verify that each unit of the codebase works as expected.
- **Scope:** <br> Very narrow, focusing on a single function or method.
- **Characteristics:**
    - Fast to run.
    - Easy to write and maintain.
    - High volume (many unit tests for different scenarios).
    - Independent of external systems (like databases or network services).
- **Example:** <br> Testing a function that calculates the total price of items in a shopping cart to ensure it returns the correct sum.

### Integration Tests

- **Purpose:** <br> Integration tests verify that different modules or services in the application work together as expected. They ensure that the interactions between different parts of the system are correct.
- **Scope:** <br> Broader than unit tests, focusing on multiple components and their interactions.
- **Characteristics:**
    - Slower than unit tests but faster than E2E tests.
    - Moderate in number (fewer than unit tests, more than E2E tests).
    - May involve actual or simulated external systems like databases or APIs.
- **Example:** <br> Testing a service that retrieves data from a database and processes it, ensuring the retrieval and processing steps work correctly together.

### End-to-End (E2E) Tests

- **Purpose:** <br> E2E tests verify the entire application flow from start to finish, ensuring that the system works as a whole in a production-like environment. They simulate real user scenarios.
- **Scope:** <br> Very broad, covering the entire application stack.
- **Characteristics:**
    - Slowest to run.
    - Most complex and expensive to write and maintain.
    - Low volume (few E2E tests due to their complexity and cost).
    - Involve all parts of the application, including the user interface, backend, and any external services.
- **Example:** <br> Testing the process of a user logging in, adding items to a shopping cart, and checking out to ensure that the complete user journey works as expected.