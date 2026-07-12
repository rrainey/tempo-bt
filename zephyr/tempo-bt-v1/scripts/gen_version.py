#!/usr/bin/env python3
# scripts/gen_version.py
# SPDX-License-Identifier: Apache-2.0

import subprocess
import sys
import os
import datetime

# Application root (parent of the scripts/ directory), so results do not
# depend on the caller's working directory
APP_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def get_git_info():
    """Get git commit hash and dirty status"""
    try:
        # Get commit hash
        commit = subprocess.check_output(
            ['git', 'rev-parse', '--short', 'HEAD'],
            stderr=subprocess.DEVNULL,
            cwd=APP_ROOT
        ).decode('utf-8').strip()

        # Check if working directory is dirty
        dirty = subprocess.call(
            ['git', 'diff-index', '--quiet', 'HEAD', '--'],
            stderr=subprocess.DEVNULL,
            cwd=APP_ROOT
        ) != 0

        if dirty:
            commit += '-dirty'

        return commit
    except:
        return 'unknown'

def get_version_from_file():
    """Read version from app.version file"""
    try:
        with open(os.path.join(APP_ROOT, 'app.version'), 'r') as f:
            return f.read().strip()
    except:
        return '0.1.0'

def main():
    if len(sys.argv) != 2:
        print("Usage: gen_version.py <output_header>")
        sys.exit(1)
        
    output_file = sys.argv[1]
    
    # Get version info
    version = get_version_from_file()
    git_commit = get_git_info()
    build_date = datetime.datetime.now(datetime.timezone.utc).strftime('%Y-%m-%d')
    
    # Generate header content
    header_content = f"""/*
 * Auto-generated version information
 * DO NOT EDIT
 */

#ifndef APP_VERSION_H
#define APP_VERSION_H

#define APP_VERSION_STRING "{version}"
#define APP_GIT_COMMIT "{git_commit}"
#define APP_BUILD_DATE "{build_date}"
#define APP_DEVICE_TYPE "V2"
#define APP_VERSION_MAJOR {version.split('.')[0]}
#define APP_VERSION_MINOR {version.split('.')[1]}
#define APP_VERSION_PATCH {version.split('.')[2]}

/* Numeric version for Tempo (1xx) vs Dropkick (0xx) differentiation */
#define APP_VERSION_NUMERIC (100 + (APP_VERSION_MAJOR * 10) + APP_VERSION_MINOR)

#endif /* APP_VERSION_H */
"""

    # Write to file only if content changed
    try:
        with open(output_file, 'r') as f:
            if f.read() == header_content:
                return  # No change needed
    except:
        pass
        
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        f.write(header_content)
    
    print(f"Generated {output_file}")

if __name__ == '__main__':
    main()