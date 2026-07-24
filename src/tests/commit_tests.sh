#!/bin/bash

cd /Users/mattastroforge/Desktop/Model

# Check if git is initialized
if [ ! -d .git ]; then
    echo "Initializing git repository..."
    git init
    
    # Configure git user (required for commits)
    git config user.name "AI Assistant"
    git config user.email "ai@example.com"
fi

# Add all test files
echo "Adding test files to git..."
git add src/tests/

# Create initial commit
echo "Creating initial commit..."
git commit -m "Add unit testing framework with timing support"

# Show status
echo ""
echo "Git status:"
git status
